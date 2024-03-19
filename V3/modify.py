import commsCopy as comms
import gymnasium as gym
from gymnasium import spaces
from gymnasium.spaces import Discrete, Dict, Box
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
import random, time, numpy as np
import time, math



def exit():
    print("Stopping Systems Safely")
    comms.SER.stop()
    quit()

class Pendulum(gym.Env):
    metadata = {'render.modes': ['human']}
    
    # !! PARAMETERS !!
    trackLength = 50 / 10
    maxSteps = 400
    
    def __init__(self, render_mode = None):
        super(Pendulum, self).__init__()
        self.observation_space = gym.spaces.Box(low=-3.4028235e+8, high=3.4028235e+8, shape=(5,), dtype=np.float32)
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        self.ALL_STEPS = 0
        self.steps = 0
        self.saveNext = False
        self.lastTheta = 0
    
    def convertTheta(self, theta):
        # theta is given with 0 and 2pi at the bottom. Let us convert it to a scale to negative.
        return theta if theta < math.pi else theta - 2 * math.pi
    
    def collectData(self, send_action = 0):
        # retrieve the state from the physical system
        comms.runCV()
        theta, timestamp = comms.SER.getAngle()
        print(theta,timestamp)
        
        # calculate angle
        theta = (theta - ZERO_ANGLE) % (2 * math.pi)

        # calculate angular velocity
        # assumes that pi/2 to 3pi/2 takes >1 frame

        if self.prevTheta > 3*math.pi/2 and theta < math.pi/2: # case 1 
            L = (theta + 2 * math.pi - self.prevTheta)
        elif theta > 3*math.pi/2 and self.prevTheta < math.pi/2:
            L = -(self.prevTheta + 2 * math.pi - theta)
        else:
            L = theta - self.prevTheta
        L = 100 * L / (timestamp - self.prevTime)
        if abs(L) > math.pi:
            L = math.copysign(1, L) * (-abs(L) % (2*math.pi))
        
        # retrieve horiztonal position
        x = comms.COMMS_INFO[1]
        
        # retrieve horizontal velocity
        v = send_action / 255

        # adjust theta to better fit our "idea" of theta
        theta = self.convertTheta(theta)
        
        # Set values for computation next turn
        self.prevTheta = theta
        self.prevTime = timestamp
        
        # X: [0, 100], lets recale it to [-5, 5]
        # V: [-1, 1]
        # sin, cos: [-1, 1]
        # L: ~[-10, 10] (realistically, most values < 3)
        
        self.lastTheta = theta
        new_obs = np.array([(x - 50)/10, v, math.sin(theta), math.cos(theta), L])
        return new_obs
    
    def getReward(self, obs):
        return self.lastTheta ** 2 #value of theta ^ 2
     
    def checkDone(self, obs):
        if obs[0] < 0 - self.trackLength/2 or obs[0] > 0 + self.trackLength/2:
            #print(f'modify.checkDone: {obs[0]}   {self.trackLength/2}')
            return True
        elif self.steps > self.maxSteps:
            #print(f'modify.checkDone: {self.steps}   {self.maxSteps}')
            return True
        elif not comms.COMMS_INFO[2]:
            #print(f'modify.checkDone: {comms.COMMS_INFO}')
            return False
        
        return False
        
    def step(self, action):
        global model, logfile
        #1. send the action to the system
            # if self.ALL_STEPS % 2048 in [0,2047]: 
            #     self.ALL_STEPS += 1
            #     self.steps += 1
            #     comms.stop()
            #     return self.reset()
        send_action = int(action[0] * 255)
        send_string = bytes([115, abs(send_action), 1 if send_action > 0 else 0])

        try:
            comms.SER.agentMove(send_string)
        except Exception as e:
            comms.SER.stop()
            print(e)
        
        #2. wait until next time to get the data
        while (time.process_time() < self.starttime + 0.08 * self.steps):
            hidrgabor = 1 #do some bogus operation to stall
        
        #3. collect the data from the environment
        # print("stepping", self.ALL_STEPS, self.steps)
        self.ALL_STEPS += 1
        self.steps += 1

        if self.ALL_STEPS % 2000 == 0: 
            self.saveNext = True

        obs = self.collectData(send_action)
        logfile.write(str(obs))
        rew = self.getReward(obs)
        logfile.write(str(rew))
        logfile.write('\n')
        done = True if self.checkDone(obs) else False
        info = {}
        
        if done:
            comms.SER.stop()

        return obs, rew, done, info, {}

    def reset(self, seed = 0):
        comms.SER.stop()

        if self.saveNext:
            print("Saving model...")
            model.save(f"fixedppo{self.ALL_STEPS//1000+4}")
            self.saveNext = False

        self.steps = 0
        self.starttime = time.process_time() + 0.1
        
        comms.resetCart()

        history = []
        c = 0
        #while True:
        while True:
            c += 1
            # if c % 20 == 0: print(f'modify.reset: waiting... {history}')
            theta, timestamp = comms.SER.getAngle()
            theta = (theta - ZERO_ANGLE) % (2 * math.pi)
            history.append(abs(self.convertTheta(theta)))
            if len(history) > 25:
                delta = 0
                for i in range(len(history)-20, len(history)):
                    delta += abs(history[i] + history[i-1])
                if delta < 0.15: #wait for pendulum to settle down
                    break
        
        time.sleep(2) #wait another 2 seconds for good measure

        comms.resetEpisode()
        
        # TODO: make this detect properly the X position before starting
        #exit()
        
        theta, timestamp = comms.SER.getAngle()
        theta = self.convertTheta(theta)
        print(theta, timestamp)
        
        val = np.array([50, 0, math.cos(theta), math.sin(theta), 0])
        
        self.prevTime = timestamp
        self.prevTheta = 0
        
        return (val, {})
        
    def close(self):
        comms.SER.stop()
        return

def testBoundariesAndReset():
    TIME_INCREMENT = 0.1
    
    print("beginning to test safety functions", comms.COMMS_INFO)

    theta, timestamp = comms.SER.getAngle()
    theta %= (2 * math.pi)

    global ZERO_ANGLE
    ZERO_ANGLE = theta

    print("bottomtheta,", theta)

    try:
        for _ in range(8):
            comms.runCV()
            time.sleep(TIME_INCREMENT)
            comms.SER.stepRight()
            
        time.sleep(0.5)

        for _ in range(8):
            comms.runCV()
            time.sleep(TIME_INCREMENT)
            comms.SER.stepLeft()

    except KeyboardInterrupt:
        comms.SER.stop()
        exit()
    print("finished testing boundaries and reset")


logfile = open("logs.txt",'w')
env = Pendulum() #gym.make('Pendulum-v1')

# # Instantiate the agent
# model = PPO(
#     "MlpPolicy",
#     env,
#     gamma=0.98,
#     use_sde=True,
#     sde_sample_freq=4,
#     learning_rate=5*(1e-4),
#     verbose=1,
#     n_steps=512,
#     stats_window_size=10
# )

model = PPO.load("fixedppo4",print_system_info=True,env=env)

def runEpisode(trial):
    global model
    print("STARTING EPISODE")
    model.learn(total_timesteps=int(100000))
    comms.SER.stop()
    print("DONE")