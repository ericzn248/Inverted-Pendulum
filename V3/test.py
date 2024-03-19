import commsCopy as comms
from time import sleep

# with comms.MotorController() as ser:
#     print(ser.get_pos())
#     sleep(0.5)
#     print(ser.get_pos())

# ser = comms.MotorController()
for i in range(10):
    print(comms.SER.getAngle())
    sleep(0.5)
# ser.setSpeed(255, 1)
# sleep(0.2)
# ser.setSpeed(0, 1)
# ser.__exit__(1,2,3)

# print(ser.get_pos())