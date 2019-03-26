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
class SetpointActual:
    setpoint: any
    actual: any

@dataclass
class JointDef:
    name: str
    absolute_angle: int
    odrive_path: List[str]
    direction: int
    joint_zero: int
    joint_limits: List[int]
    gear_ratio: float
    cpr: int
    can_node_id: int
    has_output_encoder: int


@dataclass
class JointConfig:
    joints: List[JointDef]


def offset_angle(raw_angle, absolute_angle):
    offset = absolute_angle - raw_angle
    if offset < 0:
        offset += 4096
    return offset


class Joint:
    cf = 4096.0/3600.0

    def __init__(self, joint_def: JointDef):
        self.joint_name = joint_def.name  # type: str
        self.joint_number = int(self.joint_name)
        self.config = joint_def
        self.initialized = False
        self.energized = False
        self.offset = SetpointActual(None, None)
        self.requested_state = SetpointActual(None, None)
        self.encoder_is_ready = SetpointActual(None, None)
        self.error = None
        self.motor_angle = None
        self.output_angle_initial = None
        self.output_angle = None
        self._cpr = None
        self.cpr_initial = None
        self._shadow_count = None
        self.shadow_count_initial = None

    def reset_state(self):
        self.initialized = False
        self.energized = False
        self.offset = SetpointActual(None, None)
        self.requested_state = SetpointActual(None, None)
        self.encoder_is_ready = SetpointActual(None, None)
        self._cpr = None
        self.cpr_initial = None
        self._shadow_count = None
        self.shadow_count_initial = None

    def calculate_offset(self, angle_override=None):
        abs_angle = self.config.absolute_angle if not angle_override else angle_override
        self.offset.setpoint = offset_angle(self.motor_angle, abs_angle)
        self.output_angle_initial = self.output_angle
        # set offset angle, check written offset angle
        # read current cpr and save
        # self.initialized = True

    def calculate_limits(self):
        initial = self.get_absolute_position(self.output_angle_initial)
        neg_limit = self.config.joint_limits[0] - initial
        pos_limit = self.config.joint_limits[1] - initial
        return neg_limit*self.multiplier, pos_limit*self.multiplier

    def get_zero_position(self):
        initial = self.get_absolute_position(self.output_angle_initial)
        if self.config.has_output_encoder == 0:
            return 0
        return -initial*self.multiplier

    def get_target_position(self, target_angle):
        zero = self.get_zero_position()
        return zero + target_angle*self.multiplier

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
    def multiplier(self):
        return self.config.gear_ratio*self.cf*self.config.direction

    @property
    def verbose_name(self):
        return 'j' + self.joint_name

    def __repr__(self):
        return "Joint{}".format(self.joint_number)

    @property
    def output_angle_valid(self):
        if self.config.has_output_encoder == 0:
            return True
        return 3600 > self.output_angle > -1

    @property
    def cpr(self):
        return self._cpr

    @cpr.setter
    def cpr(self, value):
        if self._cpr is None:
            self.cpr_initial = value
        self._cpr = value

    @property
    def shadow_count(self):
        return self._cpr

    @shadow_count.setter
    def shadow_count(self, value):
        if self._shadow_count is None:
            self.shadow_count_initial = value
        self._shadow_count = value

    @property
    def current_joint_position(self):
        return self.get_absolute_position(self.output_angle)

    def get_absolute_position(self, angle):
        zero = self.config.joint_zero
        if angle >= zero:
            abs_angle = angle - zero
        else:
            abs_angle = 3599 - (zero - angle)
        if abs_angle < 1800:
            return abs_angle
        return abs_angle - 3599

class RoboticArm:
    def __init__(self):
        self.joints = self.initialize_joints()

    def joint(self, joint_name: str) -> Joint:
        joint_numbers = [j.verbose_name for j in self.joints]
        idx = joint_numbers.index(joint_name)
        return self.joints[idx]

    def search_by_can_node(self, node_id: int) -> Joint:
        node_ids = [j.config.can_node_id for j in self.joints]
        idx = node_ids.index(node_id)
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
