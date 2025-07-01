from roboclaw_driver import Roboclaw
import numpy as np
from time import sleep

baud_rate = 115200
dev_name1 = "/dev/ttyAMA0"  # change


robo = Roboclaw(dev_name1, baud_rate)
addresses = [int(128), int(129), int(130)]  # change
gear_ratio = 26.9
wheel_radius = 0.19/2
ticks_per_rev = 752
conversion_factor = 1/50
accel = int(16383/2)

print("Starting motor drives")

try:
    robo.Open()
    print("Successfully opened serial communications")
except Exception as e:
    print("Could not connect to Roboclaw: %s", e)
    exit(0)


for address in addresses:
    try:
        print(f"Attempting to talk to motor controller {address} through serial port {dev_name1} at a {baud_rate} baud_rate.")
        version = robo.ReadVersion(address)
        print(f"response for RC at {address}: {version}")
        if version[0]:
            print(f"Roboclaw Version: {repr(version[1])}")
        else:
            print("Could not get version from Roboclaw")
            exit(0)
    except Exception as e:
        print("Could not connect to Roboclaw: %s", e)
        exit(0)
    robo.ForwardM1(address, 0)
    robo.ForwardM2(address, 0)
    robo.ResetEncoders(address)

# raise SystemExit
for i in range(10):
    print(f"Waiting {i}/10")
    sleep(1)

def vel_to_qpps(vel):
    return int(vel * gear_ratio * conversion_factor * ticks_per_rev / (2 * np.pi * wheel_radius)) 

for address in addresses:
    if(address == 130):
        SPEED = -1
    else:
        SPEED = 1
        
    print(f"Testing address {address}")
    qpps = vel_to_qpps(SPEED)
    sleep(1)
    print("M1")
    robo.DutyAccelM1(address, accel, qpps)
    sleep(1)
    robo.ForwardM1(address, 0)
    sleep(1)
    print("M2")
    robo.DutyAccelM2(address, accel, qpps)
    sleep(1)
    robo.ForwardM2(address, 0)
    sleep(1)
    robo.ResetEncoders(address)