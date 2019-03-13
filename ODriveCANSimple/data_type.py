import struct


class ODriveCANDataType:
    bits = None
    format = None
    parser = None

    def __init__(self, value):
        assert self.bits is not None
        assert self.format is not None
        assert self.parser is not None
        self.value = self.parser(value)

    def pack(self):
        pass

    @classmethod
    def unpack(cls, values):
        pass

class SignedInt32(ODriveCANDataType):
    bits = 4
    format = '<i'
    parser = int

    def pack(self):
        return list(struct.pack(self.format, self.value))

    @classmethod
    def unpack(cls, values):
        assert len(values) == cls.bits
        return struct.unpack(cls.format, bytearray(values))[0]


class UnsignedInt32(SignedInt32):
    bits = 4
    format = '<I'
    parser = int

class SignedInt16(SignedInt32):
    bits = 2
    format = '<h'
    parser = int

class FloatIEEE754(SignedInt32):
    bits = 4
    format = '<f'
    parser = float
