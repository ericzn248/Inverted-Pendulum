import serial
import serial.tools.list_ports
from time import sleep
import struct
import math
import keyboard
import jtracking as t

DRY_RUN = False
t.setup()

COMMS_INFO = [50, 0, True] #xloc, angle


if DRY_RUN:
    #doest need the physical system
    def moveRight():
        COMMS_INFO[1] += 5

    def moveLeft():
        COMMS_INFO[1] -= 5

    def agentMove(command): #USE THIS WITH CAUTION
        speed = command[1]
        dir = command[2]
        if dir == 0: dir -= 1
        COMMS_INFO[1] += (speed * dir * 5) / 255 

    def stop():
        sleep(0.5)

    def getAngle():
        return 0, 0
    
else:
    ports = sorted([port.name for port in serial.tools.list_ports.comports()])
    print(ports)
    port = None
    for p in ports:
        # windows users beware: you might need to hardcode this?!?!
        if "usbserial" in p or "usbmodem" in p or "COM" in p:
            port = p
    if port is None:
        print("No port found")
        quit()
    print(port)
    ser = serial.Serial(port, 115200, timeout=5)


    """
    command:
    b"a" -> communicate that script is ready
    b"p" -> poll current position/time, 
            arduino will send back current position as float (use struct.unpack('f', ...)[0])
            time will be 4 byte float in little endian
    b"s[speed: 1 byte][direction: 1 byte]" -> set speed, then direction as bool (0 for backward, anything else for forward) eg b"s\xff\x01"
    """
    try:
        while not ser.in_waiting:
            sleep(0.01)
        ser.write(b"a")
        ser.read(ser.in_waiting)
        print("initialized")
        ser.write(b"p")
        sleep(0.1)
        if ser.in_waiting:
            print(f"waiting: {ser.in_waiting}")
            angle = struct.unpack("f", ser.read(4))[0]
            time = int.from_bytes(ser.read(4), byteorder="little")
            ser.write(b"p")
            print("current position (radians from start):", angle)
            print("time (ms):", time)
    
        def moveRight():
            ser.write(b"s\xff\x01")
            sleep(0.1)
            ser.write(b"s\x00\x00")

        def moveLeft():
            ser.write(b"s\xff\x00")
            sleep(0.1)
            ser.write(b"s\x00\x00")

        def agentMove(command): #USE THIS WITH CAUTION
            ser.write(command)

        def stop():
            ser.write(b"s\x00\x00")

        def getAngle():
            ser.write(b"p")
            sleep(0.01)
            if ser.in_waiting:
                # print(f"waiting: {ser.in_waiting}")
                angle = struct.unpack("f", ser.read(4))[0]
                time = int.from_bytes(ser.read(4), byteorder="little")
                # print("current position (radians from start):", angle % (math.pi * 2))
                # print("time (ms):", time)
            return angle, time

        if __name__ == "__main__":
            while True:
                if keyboard.is_pressed("left arrow"):
                    ser.write(b"s\xff\x00")

                elif keyboard.is_pressed("right arrow"):
                    ser.write(b"s\xff\x01") 
                
                else:
                    ser.write(b"s\x00\x00")

    # DO NOT CHANGE

    except KeyboardInterrupt:
        # ser.write(b"000f\n")
        ser.write(b"s\x00\x00")
        ser.close()

try:
    def resetEpisode():
        COMMS_INFO[2] = True

    def boundaryCheck(xavg):
        #checks if the x-position of the cart is out of bounds. Stop it if true.
        if xavg < 20 or xavg> 75:
            COMMS_INFO[2] = False
            stop()
            resetCart()

    def resetCart():
        cartLoc = COMMS_INFO[1]
        print("re-centering cart")
        middle = 50
        if(cartLoc==-1):
            print("system fail")
            return
        while(abs(cartLoc-middle) > 4):
            if cartLoc < middle: 
                moveRight()
            elif cartLoc > middle: 
                moveLeft()
            if DRY_RUN:
                cartLoc = COMMS_INFO[1]
                COMMS_INFO[1] = cartLoc
            else:
                cartLoc = t.getPoint(t.red)[0]
                COMMS_INFO[1] = cartLoc
        print("done")

    def runCV(): 
        #get all relevant locations with CV
        if DRY_RUN:
            cartLoc = COMMS_INFO[1]
        else:
            cartLoc = t.getPoint(t.red)[0]
            #print("cartlochere: ", cartLoc)
        COMMS_INFO[1] = cartLoc
        #check if we are out of bounds
        boundaryCheck(cartLoc)
except:
    stop()
    print("ERROR")
    ser.write(b"s\x00\x00")
    ser.close()
    exit()

# ****[ WARN:1@507.233] global cap_msmf.cpp:471 `anonymous-namespace'::SourceReaderCB::OnReadSample videoio(MSMF): OnReadSample() is called with error status: -1072873822
# [ WARN:1@507.270] global cap_msmf.cpp:483 `anonymous-namespace'::SourceReaderCB::OnReadSample videoio(MSMF): async ReadSample() call is failed with error status: -1072873822
# [ WARN:0@507.363] global cap_msmf.cpp:1759 CvCapture_MSMF::grabFrame videoio(MSMF): can't grab frame. Error: -1072873822
# Traceback (most recent call last):
#   File "C:\Users\veter\OneDrive\Documents\CS\school\CS12\pendulum\Solving-the-Classic-Inverted-Pendulum-with-PPO-Machine-Learning-and-Computer-Vision\slow_stuff\simple_run.py", line 28, in <module>
#     RL.runEpisode()
#   File "C:\Users\veter\OneDrive\Documents\CS\school\CS12\pendulum\Solving-the-Classic-Inverted-Pendulum-with-PPO-Machine-Learning-and-Computer-Vision\slow_stuff\RL.py", line 139, in runEpisode
#     comms.runCV()
#   File "C:\Users\veter\OneDrive\Documents\CS\school\CS12\pendulum\Solving-the-Classic-Inverted-Pendulum-with-PPO-Machine-Learning-and-Computer-Vision\slow_stuff\comms.py", line 49, in runCV
#     cartLoc = t.getPoint(t.red)[0]
#   File "C:\Users\veter\OneDrive\Documents\CS\school\CS12\pendulum\Solving-the-Classic-Inverted-Pendulum-with-PPO-Machine-Learning-and-Computer-Vision\slow_stuff\jtracking.py", line 119, in getPoint
#     blur = getBlur()
#   File "C:\Users\veter\OneDrive\Documents\CS\school\CS12\pendulum\Solving-the-Classic-Inverted-Pendulum-with-PPO-Machine-Learning-and-Computer-Vision\slow_stuff\jtracking.py", line 65, in getBlur
#     img = cv2.resize(img, (800, 450))
# cv2.error: OpenCV(4.8.1) D:\a\opencv-python\opencv-python\opencv\modules\imgproc\src\resize.cpp:4062: error: (-215:Assertion failed) !ssize.empty() in function 'cv::resize'