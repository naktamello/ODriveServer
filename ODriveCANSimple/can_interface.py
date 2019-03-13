import struct
from collections import namedtuple
from typing import List

import ODriveCANSimple.enums as enums
from ODriveCANSimple.exceptions import *

PAYLOAD_SIZE = 8


def shift_right(value, bytes_to_shift):
    return (value >> 8 * bytes_to_shift) & 0xFF


def as_ascii(byte):
    return "{0:0{1}x}".format(byte, 2)


def unpack_ieee754_float(val, big_endian=False):
    endian = '<'
    if big_endian:
        endian = '>'
    return list(struct.pack(endian + 'f', val))


class ODriveCANCommand:
    def __init__(self, name, command_code, *params, call_and_response=False):
        self.name = name
        self.command_code = command_code
        self._params = tuple(params)
        self.call_and_response = call_and_response
        # TODO optional params

    @property
    def params(self):
        if self.call_and_response:
            return ()
        return self._params


SUPPORTED_COMMANDS = [
    ODriveCANCommand('setpos', enums.MSG_SET_POS_SETPOINT, enums.DataType.INT32),
    ODriveCANCommand('encoder', enums.MSG_GET_ENCODER_COUNT, enums.DataType.INT32, enums.DataType.INT32,
                     call_and_response=True),
    ODriveCANCommand('state', enums.MSG_SET_AXIS_REQUESTED_STATE, enums.DataType.INT32)
]


def map_data_type_to_serializer(data_type: enums.DataType):
    if data_type == enums.DataType.INT32 or data_type == enums.DataType.UINT32:
        return 'add_int32'
    if data_type == enums.DataType.INT16:
        return 'add_int16'
    if data_type == enums.DataType.FLOAT:
        return 'add_float32'
    assert False

def map_data_type_to_deserializer(data_type: enums.DataType):
    if data_type == enums.DataType.INT32:
        return 'deserialize_int32'
    if data_type == enums.DataType.UINT32:
        return 'deserialize_uint32'
    if data_type == enums.DataType.INT16:
        return 'deserialize_int16'
    if data_type == enums.DataType.FLOAT:
        return 'deserialize_float32'
    assert False

def deserialize_int32(payload):
    value = struct.unpack('<i', bytearray(payload[:4]))[0]
    return value, payload[4:]

def deserialize_uint32(payload):
    assert False

def deserialize_int16(payload):
    assert False

def deserialize_float32(payload):
    assert False

def parse_data_type(value, data_type: enums.DataType):
    if data_type == enums.DataType.INT32 or data_type == enums.DataType.UINT32 or data_type == enums.DataType.INT16:
        return int(value)
    if data_type == enums.DataType.FLOAT:
        return float(value)
    assert False


def find_command_definition_by_name(cmd_name):
    names = [cmd.name for cmd in SUPPORTED_COMMANDS]
    idx = names.index(cmd_name)
    return SUPPORTED_COMMANDS[idx]


def find_command_definition_by_code(cmd_code):
    codes = [cmd.command_code for cmd in SUPPORTED_COMMANDS]
    idx = codes.index(cmd_code)
    return SUPPORTED_COMMANDS[idx]


MethodValuePair = namedtuple('MethodValuePair', ['method', 'value'])


class ODriveCANInterface:
    def __init__(self):
        self.known_commands = [cmd_def.name for cmd_def in SUPPORTED_COMMANDS]

    def process_command(self, command_tokens):
        node_id = int(command_tokens[0])
        if node_id > 0x3f:
            raise ODriveCANInvalidNodeID
        if command_tokens[1] not in self.known_commands:
            raise ODriveCANUnsupportedCommand
        cmd_def = find_command_definition_by_name(command_tokens[1])
        payload = self.parse_params(cmd_def, command_tokens[2:])
        packet = ODriveCANPacket(node_id, cmd_def.command_code)
        for param in payload:
            getattr(packet, param.method)(param.value)
        return encode_sCAN(packet)

    def parse_params(self, cmd_def: ODriveCANCommand, params) -> List[MethodValuePair]:
        if len(params) != len(cmd_def.params):
            raise ODriveCANSignatureMismatch
        payload = []
        for param, data_type in zip(params, cmd_def.params):
            method_name = map_data_type_to_serializer(data_type)
            parsed = parse_data_type(param, data_type)
            payload.append(MethodValuePair(method_name, parsed))
        return payload

    def process_response(self, response_string):
        node_id, cmd_code, payload = decode_sCAN(response_string)
        cmd_def = find_command_definition_by_code(cmd_code)  # type: ODriveCANCommand
        assert cmd_def.call_and_response
        return self.parse_response(cmd_def, payload)

    def parse_response(self, cmd_def: ODriveCANCommand, payload: List[int]):
        values = []
        for data_type in cmd_def._params:
            function_name = map_data_type_to_deserializer(data_type)
            value, payload = globals()[function_name](payload)
            values.append(value)
        return values

class ODriveCANPacket:
    def __init__(self, node_id, msg_id):
        self.node_id = node_id
        self.msg_id = msg_id
        self.payload = [0x00] * 8
        self.msg_ptr = 0

    def add_int32(self, val):
        self.check_payload_size()
        ptr = self.msg_ptr
        for i in range(4):
            self.payload[ptr + i] = shift_right(val, i)
        self.msg_ptr += 4

    def add_int16(self, val):
        self.check_payload_size()
        ptr = self.msg_ptr
        for i in range(2):
            self.payload[ptr + i] = shift_right(val, i)
        self.msg_ptr += 2

    def add_float32(self, val):
        byte_list = unpack_ieee754_float(val)
        self.check_payload_size()
        ptr = self.msg_ptr
        for i, b in enumerate(byte_list):
            self.payload[ptr + i] = b
        self.msg_ptr += 4

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
        return None
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
    return frame_id + can_id_ascii + dlc + data + cr


if __name__ == '__main__':
    packet = ODriveCANPacket(0x03, 0x1f)
    packet.add_int16(30000)
    packet.add_float32(1.11)
    packet_string = encode_sCAN(packet)
    node_id, cmd_id, payload = decode_sCAN(packet_string)
    print(hex(node_id), hex(cmd_id), [hex(p) for p in payload])
