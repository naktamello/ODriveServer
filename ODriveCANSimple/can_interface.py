import struct

PAYLOAD_SIZE = 8


class ODriveCANPacketException(Exception):
    pass


def shift_by_byte(value, bytes_to_shift):
    return (value >> 8 * bytes_to_shift) & 0xFF


def as_ascii(byte):
    return "{0:0{1}x}".format(byte, 2)

def unpack_ieee754_float(val, big_endian=False):
    endian = '<'
    if big_endian:
        endian = '>'
    return list(struct.pack(endian+'f', val))


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
            self.payload[ptr + i] = shift_by_byte(val, i)
        self.msg_ptr += 4

    def add_int16(self, val):
        self.check_payload_size()
        ptr = self.msg_ptr
        for i in range(2):
            self.payload[ptr + i] = shift_by_byte(val, i)
        self.msg_ptr += 2

    def add_float32(self, val):
        byte_list = unpack_ieee754_float(val)
        self.check_payload_size()
        ptr = self.msg_ptr
        for i, b in enumerate(byte_list):
            self.payload[ptr+i] = b
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
    cmd_id = (cmd_id_int & 0b00000011111)
    return node_id, cmd_id, payload

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
