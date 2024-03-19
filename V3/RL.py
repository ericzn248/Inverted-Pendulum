import comms
import time
import math
import torch
time.sleep(1) #Wait 1 seconds to ensure that its set up
import gymnasium as gym
from gymnasium.spaces import Discrete, Dict, Box
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.buffers import RolloutBuffer
from stable_baselines3.common.utils import obs_as_tensor
import random
import numpy as np
from gymnasium import spaces

VERBOSE = False

def exit():
    print("Stopping Systems Safely")
    comms.stop()
    quit()

class Pendulum(gym.Env):
    metadata = {'render.modes': ['human']}
    pos = 0
    bound = 50
    steps = 0
    def __init__(self, render_mode = None):
        super(Pendulum, self).__init__()
        
        self.observation_space = gym.spaces.Box(low=-3.4028235e+8, high=3.4028235e+8, shape=(5,), dtype=np.float32)
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        #self.env.action_space

    maxSteps = 0
    def step(self, action):
        self.pos += action
        self.steps += 1
        
        if (self.steps > 1000):
            done = True

        obs, rew, done, info  = (0, 0, 0, 0) #self.pastObs[max(0, len(self.pastObs)-5)]

        if (abs(obs[0]) > 10):
            done = True

        return obs, rew, done, info, {}

    def reset(self, seed = 0):
        self.pos = 0
        self.steps = 0
        val = (0, 0, 0, 0)

        return (val, {})
    
    def close (self):
        exit()

env = Pendulum()

model = PPO(
    "MlpPolicy",
    env,
    gamma=0.98,
    use_sde=True,
    sde_sample_freq=4,
    learning_rate=5*(1e-4),
    verbose=1,
)

buffer = RolloutBuffer(buffer_size=10000, observation_space=env.observation_space, action_space=env.action_space)

TIME_INCREMENT = 0.1 #seconds

def testBoundariesAndReset():
    print("beginning to test safety functions", comms.COMMS_INFO)

    theta, timestamp = comms.getAngle()
    theta %= (2 * math.pi)

    global ZERO_ANGLE
    ZERO_ANGLE = theta

    print("bottomtheta,", theta)

    try:
        for _ in range(8):
            comms.runCV()
            time.sleep(TIME_INCREMENT)
            comms.moveRight()
            
        time.sleep(0.5)

        for _ in range(8):
            comms.runCV()
            time.sleep(TIME_INCREMENT)
            comms.moveLeft()

    except KeyboardInterrupt:
        comms.stop()
        exit()
    
    print("finished testing boundaries and reset")

def runEpisode(epNum):
    episodePackage = []
    try:
        comms.resetEpisode()
        buffer.reset()
        frame = 0
        prevTheta = 0
        _, prevTime = comms.getAngle()
        model.policy.set_training_mode(True)
        model.policy.reset_noise(model.env.num_envs)
        prevDir = 0
        
        last_obs = np.array([[50, 0, math.cos(0), math.sin(0), 0]]) # this is the starting position per episode
        last_done = False #representing if the episode is over yet

        episode_active = True
        totalReward = 0

        startTime = time.process_time() + 0.1

        thetaMax, thetaMin = -100, 100
        
        

        while episode_active: #this is the training loop
            if time.process_time() < startTime + frame * 0.06:
                continue

            # do we really need to add more noise???
            model.policy.reset_noise() # model.env.num_envs

            #NOTE TO SELF: Can move these some operations for after episode to save tme
            #https://github.com/DLR-RM/stable-baselines3/blob/master/stable_baselines3/common/on_policy_algorithm.py

            
            with torch.no_grad():
                obs_tensor = obs_as_tensor(last_obs,'cuda') #can change later if we have GPUs, but the slowing factor is definitely the robot not the CPU
                actions, values, log_probs = model.policy(obs_tensor)

            actions = actions.cpu().numpy()
            clipped_actions = np.clip(actions, model.env.action_space.low, model.env.action_space.high) # could be wrong

            #------------------------------------------------------------
            send_action = int(clipped_actions[0][0] * 255)
            send_string = bytes([115, abs(send_action), 1 if send_action > 0 else 0])
            
            if VERBOSE:
                print("String to send", send_string)        
                print(b"s\x00\x00")     
            
            comms.agentMove(send_string)

            #get the state
            comms.runCV()
            theta, timestamp = comms.getAngle()
            theta -= ZERO_ANGLE
            theta %= (2 * math.pi)

            if (prevTheta > math.pi and theta < math.pi):
                delta = (theta + 2 * math.pi - prevTheta)
            elif (theta > math.pi and prevTheta < math.pi):
                delta = -(prevTheta + 2 * math.pi - theta)
            else:
                delta = theta - prevTheta
            delta = 100 * delta / (timestamp - prevTime)
            #print(theta, prevTheta, delta)
            #use congruence
            if abs(delta) > math.pi:
                delta = math.copysign(1, delta) * (-abs(delta) % (2*math.pi))
            L = delta

            prevTheta = theta
            prevTime = timestamp

            x = comms.COMMS_INFO[1]
            v = send_action / 255

            if VERBOSE:
                print(f"X {x}, Theta {theta}, L {L} @ timestamp {timestamp}")

            new_obs = np.array([[x, v, math.sin(theta), math.cos(theta), L]])
            
            if theta > math.pi:
                r_theta = 2 * math.pi - theta
            else:
                r_theta = theta

            if theta > math.pi:
                thetaMin = min(thetaMin, theta)    
            else:
                thetaMax = max(theta, thetaMax)

            rewards = r_theta ** 2  + L ** 2 # Put reward function here

            # training mode, eval, x scael

            # if r_theta < 0.8: 
            #     r_theta = 0

            # # TEST STUPID ENVIRONMENT RN:
            # rewards = (100-abs(25 - x)) / 100

            totalReward += rewards
            done = False # Put done function here

            episodePackage.append(([x, v, theta, L], rewards))

            if not comms.COMMS_INFO[2]:
                print("uh oh! cart ran out of bounds... terminating episode")
                rewards -= 15
                last_done = True
                buffer.add(last_obs, actions, rewards, last_done, values, log_probs)
                break

            if frame > 200:
                comms.stop()
                time.sleep(1)
                comms.resetCart()
                last_done = True
                buffer.add(last_obs, actions, rewards, last_done, values, log_probs)
                break
            #------------------------------------------------------------
            buffer.add(last_obs, actions, rewards, last_done, values, log_probs)

            last_obs = new_obs
            last_done = done

            frame += 1   
            
            if frame % 20 == 0:
                print('*', end="", flush=True)

            #reward it better
            #better reset

        with torch.no_grad():
            values = model.policy.predict_values(obs_as_tensor(new_obs,'cuda'))  # type: ignore[arg-type]
        buffer.compute_returns_and_advantage(last_values=values, dones=done)
        print("finished computing episode, total reward:", totalReward)
        print(f"NThetamax {thetaMax}, thetamin {thetaMin}")
        print(frame / (time.process_time() - startTime))
        if epNum % 10 == 0: model.save(f"ppo{epNum}")
        #print(model.logger.dump())

        return episodePackage

    except KeyboardInterrupt:
        comms.stop()
        exit()