from time import sleep
import odrive.enums as ODRV

def offset_angle(raw_angle, absolute_angle):
    offset = absolute_angle - raw_angle
    if offset < 0:
        offset += 4096
    return offset


class Joint:
    def __init__(self, joint_name, joint_config):
        self.joint_name = joint_name
        self.joint_number = int(joint_name)
        # self.odrv = get_odrv(joint_config['odrive_path'][0])
        self.odrv = None
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
