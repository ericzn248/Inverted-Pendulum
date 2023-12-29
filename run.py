import comms
import time
time.sleep(1) #Wait 1 seconds to ensure that its set up

import gymnasium as gym
from gym.spaces import Discrete, Dict, Box
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
import random
import numpy as np
from gym import spaces

class Pendulum(gym.Env):
    metadata = {'render.modes': ['human']}
    pos = 0
    bound = 50
    steps = 0
    def __init__(self, render_mode = None):
        super(Pendulum, self).__init__()
        self.env = CartPole() #gym.make("", render_mode = render_mode)s
        
        self.observation_space = gym.spaces.Box(low=-3.4028235e+8, high=3.4028235e+8, shape=(5,), dtype=np.float32)
        self.action_space = gym.spaces.Box(low=-3.4028235e+8, high=3.4028235e+8, shape=(5,), dtype=np.float32)
        
        #self.env.action_space
    
    maxSteps = 0
    pastObs = []
    def step(self, action):
        pckt = self.env.step(action)
        
        self.pastObs.append(pckt)
        
        self.pos += action
        self.steps += 1
        
        if (self.steps > 1000):
            done = True

        obs, rew, done, info  = self.pastObs[max(0, len(self.pastObs)-5)]

        if (abs(obs[0]) > 10):
            done = True

        return obs, rew, done, info, {}

    def reset(self, seed = 0):
        self.pos = 0
        self.steps = 0
        val = self.env.reset()

        self.pastObs = []

        return (val, {})
    
    def render(self, mode='human'):
        return self.env.render()
    
    def close (self):
        self.env.close()

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

try:
    #comms.moveRight()
    # print("starting")
    # comms.moveRight()
    # comms.moveRight()
    def runFrame(obs):
        action, _ = model(obs)
        

        
    
except KeyboardInterrupt:
    comms.stop()