# import all needed libraries
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
import random, math, time, numpy as np

# import our own code
import comms
from stable_baselines3.common.logger import configure

BASE_TIME = comms.BASE_TIME

LOAD_PREVIOUS = True # should we load a save file?
MODEL_NUM = [0,29][int(LOAD_PREVIOUS)]
PREVIOUS_MODEL = 'flipdown29.zip'

# overrides default exit function to stop the pendulum before exit
def exit():
    print("Stopping Systems Safely")
    comms.SER.stop()
    quit()

# physical environment interface
class Flipdown(gym.Env):    
    # !! PARAMETERS !!
    trackLength = 50 / 10
    maxSteps = 200

    lastVelocity = 0
    VEL_CHANGE_MAX = 160
    
    def __init__(self, render_mode = None):
        super(Flipdown, self).__init__()
        self.observation_space = gym.spaces.Box(low=-3.4028235e+8, high=3.4028235e+8, shape=(5,), dtype=np.float32)
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        self.ALL_STEPS = 0
        self.steps = 0
        self.saveNext = False
        self.lastTheta = 0
        self.stopSuccess = False
    
    def convertTheta(self, theta):
        # theta is given with 0 and 2pi at the bottom. Let us convert it to a scale to negative.
        return theta if theta < math.pi else theta - 2 * math.pi
    
    def collectData(self):
        # retrieve the state from the physical system
        done = comms.runCV()
        
        thetaNOMOD, timestamp = comms.SER.getAngle()
        # print(theta, timestamp)
        
        # massage theta value to reasonable range; here, theta is 0-2pi, clockwise, with 0 and 2pi at the bottom
        theta = (thetaNOMOD - comms.ZERO_ANGLE) % (2 * math.pi)
        L = thetaNOMOD - self.prevTheta
        
        # retrieve horiztonal position
        x = comms.COMMS_INFO[1]
        
        # retrieve horizontal velocity
        v = self.lastVelocity / 255

        # adjust theta to better fit our "idea" of theta
        theta = self.convertTheta(theta)
        
        # Set values for computation next turn
        self.prevTheta = thetaNOMOD
        self.prevTime = timestamp
        
        # X: [0, 100], lets recale it to [-5, 5]
        # V: [-1, 1]
        # sin, cos: [-1, 1]
        # L: ~[-10, 10] (realistically, most values < 3)
        
        self.lastTheta = theta
        new_obs = np.array([(x - 50)/10, v, math.sin(theta), math.cos(theta), L])
        
        if type(done) != bool: 
            exit()
            
        done |= self.checkDone(new_obs)

        return new_obs, done
    
    # reward function
    def getReward(self, obs):
        return (5-abs(obs[0]))/(1+5*abs(self.lastTheta))
     
    # check if episode has terminated
    def checkDone(self, obs): #x,v,sin,cos,L
        if self.steps > self.maxSteps:
            #print(f'modify.checkDone: {self.steps}   {self.maxSteps}')
            return True
        if not comms.COMMS_INFO[2]:
            #print(f'modify.checkDone: {comms.COMMS_INFO}')
            return False
        if abs(obs[0])<0.5 and abs(obs[4])<0.03 and abs(self.lastTheta)<0.03:
            comms.testImage()
            print("bozo")
            self.stopSuccess = True
            return True
        
        return False
        
    def step(self, action):
        global model, logfile

        logfile.write(f"[Time: {time.time() - BASE_TIME}] Stepping #{self.steps}, Total: {self.ALL_STEPS}\n")

        dv = int(action[0] * self.VEL_CHANGE_MAX)
        # self.lastActs.append(action[0])

        self.lastVelocity = max(-255, min(self.lastVelocity + dv, 255))
        # send_string = bytes([115, abs(send_action), 1 if send_action > 0 else 0]) [deprecated]

        logfile.write(f"Preparing to send action: {self.lastVelocity}\n")

        try:
            comms.SER.setSpeed(abs(self.lastVelocity), 1 if self.lastVelocity > 0 else 0)
            # comms.SER.agentMove(send_string) [deprecated]
        except Exception as e:
            print(e)
            exit()

        logfile.write(f"[Time: {time.time() - BASE_TIME}] Change: {dv}, Send Action: {self.lastVelocity}\n")

        
        #2. wait until next time to get the data
        while (time.time() < self.starttime + 0.05 * (self.steps + 1)):
            hidrgabor = 1 #do some bogus operation to stall
        
        #3. collect the data from the environment
        # print("stepping", self.ALL_STEPS, self.steps)
        self.ALL_STEPS += 1
        self.steps += 1
        #print(self.ALL_STEPS, time.time(), time.time(), comms.COMMS_INFO)

        if self.ALL_STEPS % 1000 == 0: 
            self.saveNext = True

        logfile.write(f"[Time: {time.time() - BASE_TIME}] Preparing to collect data\n")

        obs, done = self.collectData()
        rew = self.getReward(obs)

        logfile.write(f"[Time: {time.time()- BASE_TIME}] Finished collecting data\n")
        logfile.write(str(obs))
        logfile.write(str(rew))
        logfile.write('\n')

        if done: 
            if self.stopSuccess:
                print("extra reward given")
                rew+=1000
                self.stopSuccess = False
                #////////////////////////////////
                # HOW DO I STOP THE RUN HERE??? #
                #////////////////////////////////
            print(f"frames per second: {self.steps/(time.time() - self.starttime)}")
            logfile.write(f"[EPISODE FINISHED] steps: {self.steps}, allsteps: {self.ALL_STEPS}, fps: {self.steps/(time.time() - self.starttime)}")
            comms.SER.stop()
        return obs, rew, done, {}, {}

    # reset; called everytime before a new episode starts
    def reset(self, seed = 0):

        print("resetting environment...")
        comms.SER.stop()

        comms.softReset()
        if self.saveNext:
            print("Saving model...")
            model.save(f"flipdown{self.ALL_STEPS//1000+MODEL_NUM}")
            self.saveNext = False
            print("Saved.") 

        comms.resetEpisode()
                
        self.steps = 0
        self.starttime = time.time() + 0.1
        theta, timestamp = comms.SER.getAngle()
        self.prevTime = timestamp
        self.prevTheta = theta

        obs, _ = self.collectData()
        #val = np.array([50, 0, math.cos(theta), math.sin(theta), 0])

        return (obs, {})
        
    def close(self):
        comms.SER.stop()
        return

logfile = open("flipdownModify.txt",'w')
env = Flipdown() #gym.make('Pendulum-v1')

if LOAD_PREVIOUS:
    model = PPO.load(PREVIOUS_MODEL,print_system_info=True,env=env)
else:
    model = PPO(
        "MlpPolicy",
        env,
        gamma=0.98,
        use_sde=True,
        sde_sample_freq=4,
        learning_rate=5*(1e-4),
        verbose=1,
        n_steps=200,
        stats_window_size=10
    )

tmp_path = "flipdown"
# set up logger
new_logger = configure(tmp_path, ["stdout", "csv", "tensorboard"])

model.set_logger(new_logger)

def runEpisodes(steps):
    global model
    model.learn(total_timesteps=int(steps))
    comms.SER.stop()
