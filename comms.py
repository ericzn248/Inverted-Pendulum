import serial
import serial.tools.list_ports
from time import sleep
import struct
import math

import keyboard

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

    # # code to flip the pendulum
    # interval = 0.2
    # while True:
    #     sleep(interval)
    #     ser.write(b"s\xff\x01")
    #     sleep(interval)
    #     ser.write(b"s\xff\x00")
    #     interval *= 1.05
    #     print(interval)
   



    def moveRight():
        ser.write(b"s\xff\x01")
        sleep(0.5)
        ser.write(b"s\x00\x00")

    def moveLeft():
        ser.write(b"s\xff\x00")
        sleep(0.5)
        ser.write(b"s\x00\x00")

    def stop():
        ser.write(b"s\x00\x00")

    def getAngle():
        ser.write(b"p")
        sleep(0.1)
        if ser.in_waiting:
            print(f"waiting: {ser.in_waiting}")
            angle = struct.unpack("f", ser.read(4))[0]
            time = int.from_bytes(ser.read(4), byteorder="little")
            ser.write(b"p")
            print("current position (radians from start):", angle % (math.pi * 2))
            print("time (ms):", time)

    while True:
        if keyboard.is_pressed("left arrow"):
            ser.write(b"s\xff\x00")

        elif keyboard.is_pressed("right arrow"):
            ser.write(b"s\xff\x01")
        
        else:
            ser.write(b"s\x00\x00")

    #moveRight()

# DO NOT CHANGE

except KeyboardInterrupt:
    # ser.write(b"000f\n")
    ser.write(b"s\x00\x00")
    ser.close()