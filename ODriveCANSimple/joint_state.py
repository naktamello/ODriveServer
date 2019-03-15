from operator import attrgetter
from time import sleep
from typing import Tuple
import os
import odrive.enums as ODRV
import yaml

cur_dir = os.path.dirname(os.path.abspath(__file__))


def offset_angle(raw_angle, absolute_angle):
    offset = absolute_angle - raw_angle
    if offset < 0:
        offset += 4096
    return offset


class Joint:
    def __init__(self, joint_name, joint_config):
        self.joint_name = joint_name
        self.joint_number = int(joint_name)
        self.axis_name = joint_config['odrive_path'][1]
        self.config = joint_config
        self.initialized = False
        self.energized = False
        self.offset_angle = None
        self.cpr = None

    def initialize(self, encoder_reading):
        self.offset_angle = offset_angle(encoder_reading, self.config['absolute_angle'])
        # set offset angle, check written offset angle
        # read current cpr and save
        self.initialized = True

    def energize(self):
        if self.initialized:
            # request state
            error = None  # check for error and state (heartbeat)
            if not error:
                self.energized = True
        # enter closed loop control

    def deenergize(self):
        self.energized = False
        # enter idle state

    def move_to(self, pos):
        # TODO check axis state is in closed loop control, no error
        if self.initialized and self.energized:
            pass

    @property
    def verbose_name(self):
        return 'j' + self.joint_name

    @property
    def absolute_pos(self):
        return 0

    @property
    def relative_pos(self):
        return 0

    def __repr__(self):
        return "Joint{}".format(self.joint_number)


class RoboticArm:
    def __init__(self):
        self.joints = self.initialize_joints()

    def joint(self, joint_number: int):
        joint_numbers = [j.joint_number for j in self.joints]
        idx = joint_numbers.index(joint_number)
        return self.joints[idx]

    @staticmethod
    def initialize_joints(skip=None):
        joints = list()
        with open(os.path.join(cur_dir, 'configs', 'joints.yaml'), encoding='utf-8') as infile:
            joint_configs = yaml.safe_load(infile)
        for joint_name, config in joint_configs.items():
            if skip and "j" + joint_name in skip:
                continue
            joints.append(Joint(joint_name, config))
        joints = sorted(joints, key=attrgetter('joint_number'))

        return tuple(joints)


if __name__ == '__main__':
    robotic_arm = RoboticArm()
