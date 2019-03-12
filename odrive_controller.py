from operator import attrgetter

import odrive
from time import sleep
import odrive.enums as ODRV
import yaml
import os
from serial import Serial

cur_dir = os.path.dirname(os.path.abspath(__file__))
uart = Serial(port="/dev/ttyJ1", baudrate=115200)

OFFSET = 417  # JOINT4


def offset_angle(raw_angle, absolute_angle):
    offset = absolute_angle - raw_angle
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
        self.joint_names = ('j1', 'j2', 'j3', 'j4', 'j5', 'j6')
        self.joint_values = dict()

    def populate_joint_angles(self):
        joint_names = list(self.joint_names)
        self.joint_values = dict()
        while len(joint_names):
            raw = receive_signal(uart)
            if ":" in raw and raw.endswith("\n"):
                name, angle1, angle2 = parse_message(raw)
                if name in joint_names and valid_amt_angle(angle1) and valid_mlx_angle(angle2):
                    self.joint_values[name] = {'amt': angle1, 'mlx': angle2}
                    joint_names.remove(name)
                else:
                    print('invalid:', name, angle1, angle2)
        print(self.joint_values)

    def get_motor_absolute_position(self, joint_name):
        return self.joint_values[joint_name]['amt']

    def get_joint_absolute_position(self, joint_name):
        return self.joint_values[joint_name]['mlx']


class ODrive:
    def __init__(self, name, serial_number):
        self.name = name
        self.serial_number = serial_number
        self.instance = None
        self.connect()

    def connect(self):
        print("connecting to {}:{}".format(self.name, self.serial_number))
        self.instance = odrive.find_any(serial_number=self.serial_number, timeout=30)


class Joint:
    def __init__(self, joint_name, joint_config):
        self.joint_name = joint_name
        self.joint_number = int(joint_name)
        self.odrv = get_odrv(joint_config['odrive_path'][0])
        self.axis_name = joint_config['odrive_path'][1]
        self.config = joint_config
        self.initialized = False
        self.energized = False
        self.offset_angle = None
        self.initial_count = 0

    def initialize(self, encoder_reading):
        self.offset_angle = offset_angle(encoder_reading, self.config['absolute_angle'])
        self.axis.encoder.config.offset = self.offset_angle
        self.axis.encoder.is_ready = True
        self.initialized = True
        # write offset value set encoder.is_ready to True and check
        # also save initial count

    def energize(self):
        if self.initialized:
            self.axis.requested_state = ODRV.AXIS_STATE_CLOSED_LOOP_CONTROL
            sleep(0.5)
            if self.axis.error == 0:
                self.energized = True
        # enter closed loop control

    def deenergize(self):
        self.axis.requested_state = ODRV.AXIS_STATE_IDLE
        self.energized = False
        # enter idle state

    def move_to(self, pos):
        # TODO check axis state is in closed loop control, no error
        if self.initialized and self.energized:
            self.axis.controller.pos_setpoint = pos

    @property
    def axis(self):
        return getattr(self.odrv.instance, self.axis_name)

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

j1, j2, j3, j4, j5, j6 = initialize_joints()

for joint_name in can_bus.joint_names:
    globals()[joint_name].initialize(can_bus.get_motor_absolute_position(joint_name))

def energize_all():
    for joint_name in can_bus.joint_names:
        globals()[joint_name].energize()

def deenergize_all():
    for joint_name in can_bus.joint_names:
        globals()[joint_name].deenergize()

def move_p1():
    j1.move_to(20000)
    j2.move_to(15000)
    j3.move_to(-30000)
    j4.move_to(10000)
    j5.move_to(-20000)
    j6.move_to(50000)

def move_p0():
    j1.move_to(0)
    j2.move_to(0)
    j3.move_to(0)
    j4.move_to(0)
    j5.move_to(0)
    j6.move_to(0)

print(j1, j2, j3, j4, j5, j6)
# angle = offset_angle(can_bus.joint_values['j4']['amt'], OFFSET)

# if __name__ == '__main__':
#     print(angle)
# print("connecting to odrive...")
# odrv0 = odrive.find_any()
# print("connected")
# odrv0.axis1.encoder.config.offset = angle
# odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
