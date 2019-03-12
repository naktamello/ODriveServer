# coding=utf-8
import odrive
from odrive.enums import *
from serial import Serial
from time import sleep
# odrv0 = odrive.find_any()
serial = Serial(port="/dev/ttyJ1", baudrate=115200)
# formula: AMT reading - odrvX.axisX.encoder.offset; (add 4096 if <0)
# OFFSET = 1165  # JOINT1
# OFFSET = 458  # JOINT2
# OFFSET = 3214  # JOINT3
# OFFSET = 417  # JOINT4
# OFFSET = 2024  # JOINT5
OFFSET = 3014  # JOINT6

def receive_signal():
    """receive signal with a non-blocking way (KuoE0)"""
    data = ""
    try:
        if serial.in_waiting != 0:
            data = serial.readline().decode()
    except Exception as e:
        error_msg = "Error reading from {0}"
        template = "An exception of type {0} occured. Arguments:\n{1!r}"
        message = template.format(type(e).__name__, e.args)
        print(error_msg, message)
        serial.reset_input_buffer()
        serial.reset_output_buffer()
    return data


def valid_angle(angle):
    return 4096 > angle > -1


def parse(msg):
    tokens = msg.strip().split(":")
    # return dict(id=int(tokens[0]), angle=int(tokens[1]))
    return dict(id=tokens[0], angle=int(tokens[1]))

def offset_angle(raw_angle):
    offset = OFFSET - raw_angle
    if offset < 0:
        offset += 4096
    return offset

if __name__ == '__main__':
    packets = []
    while True:
        raw = receive_signal()
        if ":" in raw and raw.endswith("\n"):
            data = parse(raw)
            if valid_angle(data['angle']):
                packets.append(data)
        # if len(packets) > 1 and packets[-1] == packets[-2]:
        if len(packets) > 1 and packets[-1]['id'] == 'joint3':
            break
    angle = offset_angle(packets[-1]['angle'])
    print(angle)
    # print("angle offset:{}".format(angle))
    print("connecting to odrive...")
    odrv0 = odrive.find_any()
    print("connected")
    odrv0.axis1.encoder.config.offset = angle
    # odrv0.axis1.encoder.count_in_cpr = angle
    odrv0.axis1.encoder.is_ready = True
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.pos_setpoint = 1
    sleep(0.5)
    odrv0.axis1.controller.pos_setpoint = 1024
    sleep(0.5)
    odrv0.axis1.controller.pos_setpoint = 2048
    sleep(0.5)
    odrv0.axis1.controller.pos_setpoint = 3072
    sleep(0.5)
    odrv0.axis1.controller.pos_setpoint = 4096
    sleep(0.5)
    odrv0.axis1.controller.pos_setpoint = 1
    sleep(1)
    odrv0.axis1.requested_state = AXIS_STATE_IDLE
