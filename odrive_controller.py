from operator import attrgetter

import odrive
from time import sleep
from odrive.enums import *
import yaml
import os
from serial import Serial

cur_dir = os.path.dirname(os.path.abspath(__file__))
uart = Serial(port="/dev/ttyJ1", baudrate=115200)

OFFSET = 417  # JOINT4


def offset_angle(raw_angle, offset):
    offset = OFFSET - raw_angle
    if offset < 0:
        offset += 4096
    return offset


class ODriveNotFound(KeyError):
    pass


def get_odrv(name):
    try:
        return globals()[name]
    except KeyError as e:
        raise ODriveNotFound(e)


def receive_signal(ser):
    data = ""
    try:
        if ser.in_waiting != 0:
            data = ser.readline().decode()
    except Exception as e:
        error_msg = "Error reading from {0}"
        template = "An exception of type {0} occured. Arguments:\n{1!r}"
        message = template.format(type(e).__name__, e.args)
        print(error_msg, message)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    return data


def valid_amt_angle(angle):
    return 4096 > angle > -1


def valid_mlx_angle(angle):
    return True
    # return 3600 > angle > -1


def parse_message(msg):
    try:
        name, angles = msg.strip().split(":")
        angle1, angle2 = angles.split("/")
        # return dict(id=int(tokens[0]), angle=int(tokens[1]))
        return name, int(angle1), int(angle2)
    except Exception:
        return None, None, None


class CanBus:
    def __init__(self):
        self.serial = Serial(port="/dev/ttyJ1", baudrate=115200)
        # self.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        self.joint_names = ['j1', 'j3', 'j4']
        self.joint_values = dict()

    def populate_joint_angles(self):
        while len(self.joint_names):
            raw = receive_signal(uart)
            if ":" in raw and raw.endswith("\n"):
                name, angle1, angle2 = parse_message(raw)
                if name in self.joint_names and valid_amt_angle(angle1) and valid_mlx_angle(angle2):
                    self.joint_values[name] = {'amt': angle1, 'mlx': angle2}
                    self.joint_names.remove(name)
                else:
                    print('invalid:', name, angle1, angle2)
        print(self.joint_values)


class ODrive:
    def __init__(self, name, serial_number):
        self.name = name
        self.serial_number = serial_number
        self.instance = None

    def connect(self):
        self.instance = odrive.find_any(serial_number=self.serial_number, timeout=10)


class Joint:
    def __init__(self, joint_name, joint_config):
        self.joint_name = joint_name
        self.joint_number = int(joint_name)
        self.odrv = get_odrv(joint_config['odrive_path'][0])
        self.axis_name = joint_config['odrive_path'][1]
        self.config = joint_config
        self.initialized = False
        self.initial_count = 0

    def initialize(self, offset):
        pass
        # write offset value set encoder.is_ready to True and check
        # also save initial count

    def energize(self):
        pass
        # enter closed loop control

    def deenergize(self):
        pass
        # enter idle state

    def move_to(self, pos):
        pass

    @property
    def absolute_pos(self):
        return 0

    @property
    def relative_pos(self):
        return 0

    def __repr__(self):
        return "Joint{}".format(self.joint_number)

def initialize_odrives():
    with open(os.path.join(cur_dir, 'configs', 'odrive.yaml'), encoding='utf-8') as infile:
        odrive_config = yaml.safe_load(infile)
    for k, v in odrive_config.items():
        globals()[k] = ODrive(name=k, serial_number=odrive_config[k]['serial_number'])


def initialize_joints(skip=None):
    joints = list()
    with open(os.path.join(cur_dir, 'configs', 'joints.yaml'), encoding='utf-8') as infile:
        joint_configs = yaml.safe_load(infile)
    for joint_name, config in joint_configs.items():
        if skip and joint_name in skip:
            continue
        print(joint_name)
        joints.append(Joint(joint_name, config))
    joints = sorted(joints, key=attrgetter('joint_number'))

    return tuple(joints)
    # read serial, block until all expected info has been obtained
    # loop through each joint and call initialize()


# odrv12 = ODrive(name=odrive_config['odrv12']['name'], serial_number=odrive_config['odrv12']['serial_number'])
# joint1 = Joint(1, odrv12)

initialize_odrives()

can_bus = CanBus()
can_bus.populate_joint_angles()

joints = initialize_joints(skip=['2', '5', '6'])
print(joints)
angle = offset_angle(can_bus.joint_values['j4']['amt'], OFFSET)

get_odrv('odrv12')
if __name__ == '__main__':
    print(angle)
    # print("connecting to odrive...")
    # odrv0 = odrive.find_any()
    # print("connected")
    # odrv0.axis1.encoder.config.offset = angle
    # odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
