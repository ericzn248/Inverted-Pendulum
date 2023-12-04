import comms
import time
time.sleep(3) #Wait five seconds to ensure that its set up
try:
    #comms.moveRight()
    # print("starting")
    for i in range(50):
        comms.moveRight()
        comms.getAngle()
        time.sleep(0.01)
        comms.moveLeft()
        comms.getAngle()
        time.sleep(0.01)
        
    
except KeyboardInterrupt:
    comms.stop()