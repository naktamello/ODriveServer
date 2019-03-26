import struct
from collections import namedtuple
from typing import List, Tuple

import ODriveCANSimple.enums as enums
from ODriveCANSimple.data_type import SignedInt32, ODriveCANDataType, UnsignedInt32, FloatIEEE754
from ODriveCANSimple.exceptions import *
from ODriveCANSimple.helper import as_ascii

PAYLOAD_SIZE = 8


class ODriveCANCommand:
    def __init__(self, name, command_code, *params, call_and_response=False, response_code=None):
        self.name = name
        self.command_code = command_code
        self._params = tuple(params)  # type: Tuple[ODriveCANDataType]
        self.call_and_response = call_and_response
        self.response_code = response_code
        # TODO optional params

    @property
    def param_defs(self):
        if self.call_and_response:
            return ()
        return self._params

    @property
    def response_defs(self):
        return self._params


SUPPORTED_COMMANDS = [
    ODriveCANCommand('setpos', enums.MSG_SET_POS_SETPOINT, SignedInt32),
    ODriveCANCommand('encoder', enums.MSG_GET_ENCODER_COUNT, SignedInt32, SignedInt32,
                     call_and_response=True),
    ODriveCANCommand('state', enums.MSG_SET_AXIS_REQUESTED_STATE, SignedInt32),
    ODriveCANCommand('roffset', enums.MSG_GET_ENCODER_OFFSET, SignedInt32, SignedInt32,
                     call_and_response=True),
    ODriveCANCommand('woffset', enums.MSG_SET_ENCODER_OFFSET, SignedInt32),
    ODriveCANCommand('heartbeat', enums.MSG_GET_ODRIVE_HEARTBEAT, UnsignedInt32, UnsignedInt32,
                     call_and_response=True, response_code=enums.MSG_ODRIVE_HEARTBEAT),
    ODriveCANCommand('settrajacc', enums.MSG_SET_TRAJ_ACCEL_LIMITS, FloatIEEE754, FloatIEEE754)
]


def find_command_definition_by_name(cmd_name):
    names = [cmd.name for cmd in SUPPORTED_COMMANDS]
    idx = names.index(cmd_name)
    return SUPPORTED_COMMANDS[idx]


def find_command_definition_by_code(cmd_code):
    try:
        codes = [cmd.command_code for cmd in SUPPORTED_COMMANDS]
        idx = codes.index(cmd_code)
    except ValueError:
        codes = [cmd.response_code for cmd in SUPPORTED_COMMANDS]
        idx = codes.index(cmd_code)
    return SUPPORTED_COMMANDS[idx]


MethodValuePair = namedtuple('MethodValuePair', ['method', 'value'])


class ODriveCANInterface:
    def __init__(self):
        self.known_commands = [cmd_def.name for cmd_def in SUPPORTED_COMMANDS]

    def process_command(self, command_tokens):
        node_id = int(command_tokens[0])
        cmd_name = command_tokens[1]
        cmd_params = command_tokens[2:]
        if node_id > 0x3f:
            raise ODriveCANInvalidNodeID
        if command_tokens[1] not in self.known_commands:
            raise ODriveCANUnsupportedCommand
        cmd_def = find_command_definition_by_name(cmd_name)
        is_remote = cmd_def.call_and_response
        packet = ODriveCANPacket(node_id, cmd_def.command_code, is_remote)
        params = []
        for param_def, param_str in zip(cmd_def.param_defs, cmd_params):  # type: ODriveCANDataType.__class__, str
            p = param_def(param_str)  # type: ODriveCANDataType
            params.append(p)
        for param in params:
            packet.add_payload(param)
        return encode_sCAN(packet)

    def process_response(self, response_string)-> Tuple[int, int, List[any]]:
        node_id, cmd_code, payload = decode_sCAN(response_string)
        cmd_def = find_command_definition_by_code(cmd_code)  # type: ODriveCANCommand
        assert cmd_def.call_and_response
        return node_id, cmd_code, self.parse_response(cmd_def, payload)

    def parse_response(self, cmd_def: ODriveCANCommand, payload: List[int]):
        values = []
        for data_type in cmd_def.response_defs:
            value = data_type.unpack(payload[:data_type.bits])
            payload = payload[data_type.bits:]
            values.append(value)
        return values


class ODriveCANPacket:
    def __init__(self, node_id, msg_id, is_remote=False):
        self.node_id = node_id
        self.msg_id = msg_id
        self.payload = [0x00] * 8
        self.msg_ptr = 0
        self.is_remote = is_remote

    def add_payload(self, data: ODriveCANDataType):
        self.check_payload_size()
        ptr = self.msg_ptr
        for i, datum in enumerate(data.pack()):
            self.payload[ptr + i] = datum
        self.msg_ptr += data.bits

    def check_payload_size(self):
        if self.msg_ptr >= PAYLOAD_SIZE:
            raise ODriveCANPacketException('payload is too long')

    @property
    def node_id_hex(self):
        return "{0:0{1}x}".format(self.node_id, 2)

    @property
    def msg_id_hex(self):
        return "{0:0{1}x}".format(self.msg_id, 3)

    @property
    def can_id(self):
        return (self.node_id << 5) + self.msg_id

    def __repr__(self):
        header = "node_id:{}, msg_id:{}".format(self.node_id_hex, self.msg_id_hex)
        payload = "payload:" + ",".join([as_ascii(byte) for byte in self.payload])
        return header + ", " + payload


DECODE_DELIMITER = [0, 1, 4, 5, 7, 9, 11, 13, 15, 17, 19, 21]


def decode_sCAN(message: str):
    if not message.startswith('t'):
        return
    _, cmd_id_hex, length, *payload_hex = [message[i:j] for i, j in zip(DECODE_DELIMITER, DECODE_DELIMITER[1:])]
    payload = [int(item, 16) for item in payload_hex]
    cmd_id_int = int(cmd_id_hex, 16)
    node_id = (cmd_id_int & 0b11111100000) >> 5
    cmd_code = (cmd_id_int & 0b00000011111)
    return node_id, cmd_code, payload


def encode_sCAN(pkt: ODriveCANPacket):
    frame_id = 't'
    can_id_ascii = "{0:0{1}x}".format(pkt.can_id, 3)
    dlc = '8'
    data = "".join([as_ascii(byte) for byte in pkt.payload])
    cr = '\r'
    if pkt.is_remote:
        frame_id = 'T'
        return frame_id + can_id_ascii + dlc + cr
    return frame_id + can_id_ascii + dlc + data + cr


if __name__ == '__main__':
    pass
