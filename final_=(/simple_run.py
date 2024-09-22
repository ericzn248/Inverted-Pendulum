import jtracking as t
import comms
import time
import tkinter as tk
import math
import cv2
import keyboard
import modify as RL


time.sleep(3) #Wait five seconds to ensure that its set up

lastTime = time.time()

try:
    print("setup complete")
    comms.runCV()
    comms.softReset()
    time.sleep(3)
    comms.testBoundariesAndReset()
    time.sleep(3)
    comms.softReset()
    time.sleep(3)
    print("beginning actual training of 1 episode")
    #we ready

    f = open('rewards.txt', 'w')

    for eps in range(100):
        print(f"\n running episode {eps}")
        episodePackage = RL.runEpisodes(eps) # each here will run 100,000 frames
        time.sleep(1)

        #render the episode for now for sanity check
        cancancan = tk.Tk()
        myCanvas = tk.Canvas(cancancan,bg='white', height=500, width=1000)
        
        totReward = 0
        for state, reward in episodePackage:
            totReward += reward
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
            # print(x1, y1, x2, y2, angle)
            
            vel = state[3]
            #print(vel)
            angle2 = 0
            if vel > 0: angle2 = angle + math.pi/2
            else: angle2 = angle - math.pi/2
            maxVel = 1
            r2 = abs((r*vel)/maxVel)
            velMarker = myCanvas.create_line(x2,y2,x2+r2*math.cos(angle2),y2+r2*math.sin(angle2),fill='green')

            vMarker = myCanvas.create_line(x1,y1, x1 + state[1] * 40, y1,fill='green')


            myCanvas.create_text(250,50,fill="darkblue",font="Times 12 italic",text=f"{state}  {reward}")
            myCanvas.update()
            time.sleep(0.08)

        myCanvas.destroy()
        cancancan.destroy()

        time.sleep(1)

        f.write(str(totReward) + "\n")

    exit()

except KeyboardInterrupt:
    comms.SER.stop()
    exit()
