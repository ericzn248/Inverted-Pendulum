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
        episodePackage = RL.runEpisode()
        time.sleep(1)

        #render the episode for now for sanity check
        cancancan = tk.Tk()
        myCanvas = tk.Canvas(cancancan,bg='white', height=500, width=1000)
        
        for state, reward in episodePackage:
            state = state[0]
            # print(state, reward)
            myCanvas.pack()
            myCanvas.delete('all')

            r = 150
            angle = state[2]
            angle += math.pi / 2
            x1, y1 = 700 - state[0] * 8, 250
            x2 = x1+r*math.cos(angle)
            y2  = y1+r*math.sin(angle)
            #print(x2, y2)
            pendulum = myCanvas.create_line(x1,y1,x2,y2,fill='red')
            angle2 = 0
            if vel > 0: angle2 = angle + math.pi/2
            else: angle2 = angle - math.pi/2
            maxVel = 10
            r2 = r*vel/maxVel
            velMarker = myCanvas.create_line(x2,y2,x2+r2*math.cos(angle2),y2+r2*math.sin(angle2),fill='green')
            
            # print(x1, y1, x2, y2, angle)
            myCanvas.create_text(250,50,fill="darkblue",font="Times 12 italic",text=f"{state}  {reward}")
            myCanvas.update()
            time.sleep(0.05)
        
        myCanvas.destroy()
        cancancan.destroy()

        time.sleep(1)

    exit()

except KeyboardInterrupt:
    comms.stop()