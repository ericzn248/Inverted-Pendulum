import comms
import time
import math
import torch
time.sleep(1) #Wait 1 seconds to ensure that its set up

import gymnasium as gym
from gym.spaces import Discrete, Dict, Box
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.buffers import RolloutBuffer
from stable_baselines3.common.utils import obs_as_tensor

import random
import numpy as np
from gym import spaces

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

print("Finished Setup")

TIME_INCREMENT = 0.1 #seconds

try:
    #Current scheme: time based
    
    # We will fill rollout buffers and collect
    buffer = RolloutBuffer(buffer_size=10000, observation_space=env.observation_space, action_space=env.action_space)

    while True:
        buffer.reset()
        frame = 0
        angleDeltas = 0 #the history of L
        deltaHistory = [0, 0, 0]
        prevTheta = 0
        model.policy.set_training_mode(False)
        model.policy.reset_noise(model.env.num_envs)
        
        last_obs = np.array([[0, 0, math.cos(0), math.sin(0), 0]]) # this is the starting position per episode
        last_done = False #representing if the episode is over yet

        episode_active = True
        while episode_active: #this is the training loop
            time.sleep(TIME_INCREMENT)
            #NOTE TO SELF: Can move these some operations for after episode to save tme
            #https://github.com/DLR-RM/stable-baselines3/blob/master/stable_baselines3/common/on_policy_algorithm.py

            model.policy.reset_noise(model.env.num_envs)
            with torch.no_grad():
                obs_tensor = obs_as_tensor(last_obs, "cpu") #can change later if we have GPUs, but the slowing factor is definitely the robot not the CPU
                actions, values, log_probs = model.policy(obs_tensor)

            actions = actions.cpu().numpy()
            clipped_actions = np.clip(actions, model.env.action_space.low, model.env.action_space.high) # could be wrong

            #------------------------------------------------------------
            #HERE WE QUERY FROM PHYSICAL SYSTEM
            send_action = int(clipped_actions[0][0] * 255)
            send_string = bytes([115, abs(send_action), 1 if send_action > 0 else 0])
            print("String to send", send_string)        
            print(b"s\x00\x00")
            comms.agentMove(send_string)

            #get the state
            print(f"Retrieving the angle @ frame {frame}")
            theta, timestamp = comms.getAngle()
            theta %= (2 * math.pi)
            
            delta = theta - prevTheta
            #use congruence
            if abs(delta) > math.pi:
                delta = math.copysign(1, delta) * (-abs(delta) % (2*math.pi))

            deltaHistory.append(delta)
            angleDeltas += (delta - deltaHistory[frame])
            L = angleDeltas/3 #average L over past 3 frames

            # CV to get the position
            x = 0
            v = 0

            print(f"Theta {theta}, L {L} @ timestamp {timestamp}")
            print(angleDeltas)

            new_obs = np.array([[x, v, math.sin(theta), math.cos(theta), L]])
            rewards = 1 # Put reward function here
            done = False # Put done function here

            #------------------------------------------------------------
        
            buffer.add(last_obs, actions, rewards, last_done, values, log_probs)

            last_obs = new_obs
            last_done = done

            frame += 1   
            
            if frame > 10:
                exit()

        with torch.no_grad():
            values = model.policy.predict_values(obs_as_tensor(new_obs, 'cpu'))  # type: ignore[arg-type]

        buffer.compute_returns_and_advantage(last_values=values, dones=done)
 
except KeyboardInterrupt:
    comms.stop()
    exit()