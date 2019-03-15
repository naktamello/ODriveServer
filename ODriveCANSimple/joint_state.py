from operator import attrgetter
from time import sleep
from typing import Tuple, List
import os
import odrive.enums as ODRV
import yaml
from dataclasses import dataclass

from dacite import from_dict

cur_dir = os.path.dirname(os.path.abspath(__file__))

@dataclass
class JointDef:
    name: str
    absolute_angle: int
    odrive_path: List[str]
    joint_limits: List[int]
    gear_ratio: float
    cpr: int
    can_node_id: int

@dataclass
class JointConfig:
    joints: List[JointDef]

def offset_angle(raw_angle, absolute_angle):
    offset = absolute_angle - raw_angle
    if offset < 0:
        offset += 4096
    return offset


class Joint:
    def __init__(self, joint_def: JointDef):
        self.joint_name = joint_def.name  # type: str
        self.joint_number = int(self.joint_name)
        self.config = joint_def
        self.initialized = False
        self.energized = False
        self.offset_angle = None
        self.motor_angle = None
        self.cpr = None

    def initialize(self, encoder_reading):
        self.offset_angle = offset_angle(encoder_reading, self.config.absolute_angle)
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
            config_dict = yaml.safe_load(infile)
            joint_configs = from_dict(JointConfig, config_dict)
        for joint_def in joint_configs.joints:
            if skip and "j" + joint_def.name in skip:
                continue
            joints.append(Joint(joint_def))
        joints = sorted(joints, key=attrgetter('joint_number'))

        return tuple(joints)


if __name__ == '__main__':
    robotic_arm = RoboticArm()
