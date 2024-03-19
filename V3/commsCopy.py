import serial
import serial.tools.list_ports
from time import sleep
import struct
import jtracking as t

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
            print(b"strange buffer: " + self.ser.read_all())

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
        self.ser.write(
            b"s"
            + speed.to_bytes(1, byteorder="big")
            + direction.to_bytes(1, byteorder="big")
        )
        # sleep(0.01) #bogo
        # resp = self.ser.read(2)
        # assert resp[0] == speed and resp[1] == direction

    def getAngle(self):
        self.ser.write(b"p")
        while self.ser.in_waiting < 8:
            pass  # this will loop infinitely if the arduino doesn't respond :(
        if self.ser.in_waiting >= 8:
            angle = struct.unpack("f", self.ser.read(4))[0]
            time = int.from_bytes(self.ser.read(4), byteorder="little")
            return angle, time
        return None, None
    
    #Helper funcs for Johnny & Eric
    def agentMove(self,string):
        self.ser.write(string)

    def stop(self):
        self.setSpeed(0,1)

    def stepRight(self):
        self.setSpeed(255,0)
        sleep(0.1)
        self.setSpeed(0,1)
    
    def stepLeft(self):
        self.setSpeed(255,1)
        sleep(0.1)
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
    print("poopoo")
    main()


#Johnny & Eric Code

COMMS_INFO = [0,50,True]
LASTFRAMESKIP = [0]
DRY_RUN = False
SER = MotorController()
t.setup()
try: 
    def resetEpisode():
        COMMS_INFO[2] = True

    def boundaryCheck(xavg):
        #checks if the x-position of the cart is out of bounds. Stop it if true.
        if xavg < 25 or xavg> 75:
            print('comms.boundaryCheck: stopping cart')
            COMMS_INFO[2] = False
            SER.stop()
            resetCart()

    def resetCart():
        cartLoc = COMMS_INFO[1]
        print("re-centering cart")
        middle = 50
        if(cartLoc==-1):
            print("system fail")
            return
        while(abs(cartLoc-middle) > 4):
            print(cartLoc)
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
        print("done")
    def runCV(): 
    #get all relevant locations with CV
        if DRY_RUN:
            cartLoc = COMMS_INFO[1]
        else:
            cartLoc = t.getPoint(t.red)[0]
            if cartLoc == -100:
                #didn't detect succesfully
                LASTFRAMESKIP[0] += 1
                print("FRAME SKIP", LASTFRAMESKIP)
                if LASTFRAMESKIP[0] < 3: #we skipped one frame, eh thats fine
                    return
            else:
                LASTFRAMESKIP[0] = 0

            #print("cartlochere: ", cartLoc)
        COMMS_INFO[1] = cartLoc

        #check if we are out of bounds
        boundaryCheck(cartLoc)
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
