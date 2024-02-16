import comms
import jtracking as t
import time
import tkinter as tk
import math
import cv2
import keyboard
import RL

time.sleep(3) #Wait five seconds to ensure that its set up

lastTime = time.time()

try:
    print("setup complete")
    comms.runCV()
    comms.resetCart()
    time.sleep(3)
    RL.testBoundariesAndReset()
    time.sleep(3)
    comms.resetCart()
    time.sleep(3)
    print("beginning actual training of 1 episode")
    #we ready

    for eps in range(100):
        print(f"\n running episode {eps}")
        RL.runEpisode()
        time.sleep(5)
    
    exit()

except KeyboardInterrupt:
    comms.stop()