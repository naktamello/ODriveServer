from serial import Serial
from time import sleep
from random import randint, shuffle

uart = Serial(port="/dev/ttyJ2", baudrate=115200)
joints = list(range(1, 7))


def make_angles():
    return randint(0, 4096), randint(0, 3600)

def make_invalid_angles():
    return 9999, randint(0, 3600)

def write_joint(joint_number, angle1, angle2):
    uart.write("j{}:{}/{}\n".format(joint_number, angle1, angle2).encode())

while True:
    shuffle(joints)
    for idx, j in enumerate(joints):
        if idx == 0:
            write_joint(j, *make_invalid_angles())
        else:
            write_joint(j, *make_angles())
        sleep(0.1)
    sleep(1)
