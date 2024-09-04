from time import sleep

import robot
import numpy as np
# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")

# Measurements
timeFor360Degrees = 6.42/2
timeFor90Degrees = timeFor360Degrees/4
tickPrRevolution = 36
wheelToWheelDistance = 40
wheelDiameter = 15

# Other variables
speed = 64
overshoot = 0
cmPrTick = (wheelDiameter * np.pi) / tickPrRevolution
driveStraight = (100 / cmPrTick) - overshoot
rotate = ((wheelToWheelDistance * np.pi) / (4 * cmPrTick)) - overshoot

k = 2
stopRotation = 0.08
ccw90 = 0.8 #+ stopRotation
cw90 = 0.8 #- stopRotation


for i in range(0,4*k):
    print(arlo.go_diff(speed-3, speed, 1, 1))
    sleep(5)

    arlo.stop()
    sleep(1)

    print(arlo.go_diff(speed-3, speed, 1, 0))
    sleep(cw90)

    arlo.stop()
    sleep(1)










































# start driving pattern
# for i in range(0, 4):
    
#     count = int.from_bytes(arlo.read_left_wheel_encoder(), byteorder="big")
#     print(int.from_bytes(arlo.read_left_wheel_encoder(), byteorder="big")-count)

#     # send a go_diff command to drive forward
#     print(arlo.go_diff(speed, speed, 1, 1))

#     while(abs(int.from_bytes(arlo.read_left_wheel_encoder(), byteorder="big")-count) < driveStraight):
#         #print(int.from_bytes(arlo.read_left_wheel_encoder(), byteorder="big"))
#         sleep(0.1)
#     print(int.from_bytes(arlo.read_left_wheel_encoder(), byteorder="big")-count)

#     # send a stop command
#     print(arlo.stop())

#     count = int.from_bytes(arlo.read_left_wheel_encoder(), byteorder="big")
    
#     # send a go_diff command to drive forward
#     print(arlo.go_diff(speed, speed, 1, -1))

#     while(abs(int.from_bytes(arlo.read_left_wheel_encoder(), byteorder="big")-count) < rotate):
#         sleep(0.1)
#     print(int.from_bytes(arlo.read_left_wheel_encoder(), byteorder="big")-count)
#     print("rotate: " + str(rotate))

#     print(count)
#     # send a stop command
#     print(arlo.stop())
    