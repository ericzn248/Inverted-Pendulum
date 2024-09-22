import serial
import serial.tools.list_ports
from time import sleep
import struct
import jtracking as t
import time, math

# reset better
# left bound tight?
logfile = open("logsComms2.txt",'w')

BASE_TIME = time.time()

BAUD = 115200
TIMEOUT = 2
LEFT,RIGHT = 1, 0

class MotorController:
    def __init__(self):
        ports = sorted([port.name for port in serial.tools.list_ports.comports()])
        port = None
        for p in ports:
            # this is for linux/mac, you'll need a COM port for windows
            if "usbserial" in p or "usbmodem" in p or "COM" in p:
                port = p
        if port is None:
            print("No port found")
            quit()
        print(port)
        self.ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
        sleep(1)  # TODO: determine whether this is necessary
        while not self.ser.in_waiting >= 3:
            self.ser.write(b"e")
            sleep(0.2)
        assert self.ser.read(3) == b"ack"
        if self.ser.in_waiting:
            print(b"strange buffer error: " + self.ser.read_all())
            self.__exit__(1,2,3)
            exit()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.setSpeed(0, LEFT)
        while not self.ser.in_waiting == 3:
            self.ser.write(b"d")
            sleep(0.01)
        assert self.ser.read(3) == b"ack"
        self.ser.close()

    def setSpeed(self, speed, direction):
        assert 0 <= speed <= 255

        logfile.write(f"[Time: {time.time() - BASE_TIME}] Ordered to move {direction} at speed {speed} \n")

        self.ser.write(
            b"s"
            + speed.to_bytes(1, byteorder="big")
            + direction.to_bytes(1, byteorder="big")
        )

        logfile.write(f"[Time: {time.time() - BASE_TIME}] Initial write: {direction} at speed {speed} \n")

        while self.ser.in_waiting < 2:
            pass
        resp = self.ser.read(2)
        assert resp[0] == speed and resp[1] == direction

        logfile.write(f"[Time: {time.time() - BASE_TIME}] Confirmed Success: move {direction} at speed {speed} \n\n")


    def getAngle(self):
        self.ser.write(b"p")
        while self.ser.in_waiting < 8:
            pass  # this will loop infinitely if the arduino doesn't respond :(
        
        angle = struct.unpack("f", self.ser.read(4))[0]
        time = int.from_bytes(self.ser.read(4), byteorder="little")
        return angle, time
    
    #Helper funcs for Johnny & Eric
    def agentMove(self,string):
        self.ser.write(string)
        # there's 2 bytes returned
        while self.ser.in_waiting < 2:
            pass
        _ = self.ser.read(2)

    def stop(self):
        self.setSpeed(0,1)

    def stepRight(self):
        st = time.time()
        self.setSpeed(135,0)
        while time.time() < st + 0.05: hidrgabor = 1
        self.setSpeed(255,0)
        while time.time() < st + 0.10: hidrgabor = 1
        self.setSpeed(135,0)
        while time.time() < st + 0.15: hidrgabor = 1
        self.setSpeed(0,0)
    
    def stepLeft(self):
        st = time.time()
        self.setSpeed(135,1)
        while time.time() < st + 0.05: hidrgabor = 1
        self.setSpeed(255,1)
        while time.time() < st + 0.10: hidrgabor = 1
        self.setSpeed(135,1)
        while time.time() < st + 0.15: hidrgabor = 1
        self.setSpeed(0,1)

def main():
    # you need to use a `with` block for this or manually call the __exit__ method/set the speed to zero
    # otherwise the pendulum cart will keep running into the wall
    with MotorController() as ser:
        # basic test code, hopefully will flip the pendulum
        duration = 0.2
        for speed in range(200, 50, -5):
            ser.setSpeed(speed, LEFT)
            sleep(duration)
            ser.setSpeed(speed, RIGHT)
            sleep(duration)
            duration *= 1.05


if __name__ == "__main__":
    print("ARE YOU SURE YOU WANT TO BE RUNNING THIS IN MAIN ? ? ?")
    main()

#Johnny & Eric Code

COMMS_INFO = [0,50,True] # IDK??, x-position, should I stop?
LASTFRAMESKIP = [0]
DRY_RUN = False
ROBUST_CALIBRATE = True
SER = MotorController()
dplace = open("dplace.txt",'w')
theta, timestamp = SER.getAngle()
theta %= (2 * math.pi)
ZERO_ANGLE = theta
print("theta displacement:", theta)

t.setup()
try: 
    def resetEpisode():
        COMMS_INFO[2] = True
    
    def convertTheta(theta):
        # theta is given with 0 and 2pi at the bottom. Let us convert it to a scale to negative.
        return theta if theta < math.pi else theta - 2 * math.pi

    def boundaryCheck(xavg):
        #checks if the x-position of the cart is out of bounds. Stop it if true.
        if xavg < 25 or xavg> 75:
            print('comms.boundaryCheck: stopping cart')
            COMMS_INFO[2] = False
            SER.stop()
            softReset()
            return True
        return False

    def softReset():
        cartLoc = COMMS_INFO[1]
        print("re-centering cart from", cartLoc)
        
        middle = 50
            
        while(abs(cartLoc-middle) > 4):
            # print(cartLoc)
            #print(cartLoc)
            if cartLoc < 0 or cartLoc > 100:
                print("frameskip")
                SER.stop()
                cartLoc = t.getPoint(t.red)[0]
                continue

            if cartLoc < middle:
                SER.stepRight()
            elif cartLoc > middle: 
                SER.stepLeft()
            if DRY_RUN:
                cartLoc = COMMS_INFO[1]
                COMMS_INFO[1] = cartLoc
            else:
                cartLoc = t.getPoint(t.red)[0]
                COMMS_INFO[1] = cartLoc
        print("finished.")

    def hardReset():
        global ZERO_ANGLE
        softReset()
        SER.stop()
        print("Performing hard-reset")
        history = []
        while True: # we will wait a maximum of 50 seconds
            # if c % 20 == 0: print(f'modify.reset: waiting... {history}')
            time.sleep(0.1)
            theta, timestamp = SER.getAngle()
            theta = (theta - ZERO_ANGLE) % (2 * math.pi)
            history.append(abs(convertTheta(theta)))

            if len(history) > 25:
                delta = 0
                total = 0
                for i in range(len(history)-20, len(history)):
                    delta += abs(history[i] - history[i-1])
                    total += abs(history[i])
                try: 
                    if delta < 0.2: #wait for settle and calibrate
                        print(f"RESET ZANGLE {ZERO_ANGLE} --> {total/20}")
                        ZERO_ANGLE = total/20
                        break
                except:
                    exit()
        print("successful reset?")
        time.sleep(2) #wait another 2 seconds for good measure
        resetEpisode()
    # test on startup to ensure functionality

    def testBoundariesAndReset():    
        print("beginning to test safety functions", COMMS_INFO)
        theta, timestamp = SER.getAngle()
        theta %= (2 * math.pi)

        global ZERO_ANGLE
        ZERO_ANGLE = theta
        print("theta displacement:", theta)

        calLeft, calRight = False, False

        try:
            for _ in range(100):
                runCV()
                if (COMMS_INFO[1] < 35 and not calLeft and ROBUST_CALIBRATE):
                    SER.stop()
                    t.setupRed()
                    calLeft = True
                    break
                SER.stepLeft()
            
            print(COMMS_INFO[1])
            
            time.sleep(0.5)

            for _ in range(100):
                runCV()
                if (COMMS_INFO[1] > 65 and not calRight and ROBUST_CALIBRATE):
                    SER.stop()
                    t.setupRed()
                    calRight = True
                    break
                SER.stepRight()
            
            print(COMMS_INFO[1])
        except KeyboardInterrupt:
            SER.stop()
            exit()
        
        print("finished testing boundaries and reset")

    
    def runCV(): 
        #get all relevant locations with CV
        if DRY_RUN: cartLoc = COMMS_INFO[1]
        else:
            cartLoc = t.getPoint(t.red)[0]
            if cartLoc == -100: #CV missed a frame
                LASTFRAMESKIP[0] += 1
                print(f"CV-Frame Skipped. Skipped {LASTFRAMESKIP[0]} frames in a row.")
                if LASTFRAMESKIP[0] < 3: #we skipped one frame, eh thats fine
                    return False
                else:
                    SER.stop()
                    SER.__exit__(1,2,3)
                    print("Too many skipped frames in a row. Aborting.")
                    exit()
            else:
                LASTFRAMESKIP[0] = 0
            #print("cartlochere: ", cartLoc)
        COMMS_INFO[1] = cartLoc

        #check if we are out of bounds
        return boundaryCheck(cartLoc)
    def testImage():
        t.testImage()

except Exception as e:
    print("ERROR")
    print(e)
    SER.stop()
    SER.__exit__(1,2,3)
    exit()

"""
command:
b"e" -> enable motor controller, wait for b"ack" response
b"d" -> disable motor controller, call before exiting, wait for b"ack" response
b"p" -> poll current position/time, 
        arduino will send back current position as float (use struct.unpack('f', ...)[0])
        time will be 4 byte float in little endian
b"s[speed: 1 byte][direction: 1 byte]" -> set speed, then direction as bool (0 for backward, anything else for forward) eg b"s\xff\x01"
"""