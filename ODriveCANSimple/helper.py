def shift_right(value, bytes_to_shift):
    return (value >> 8 * bytes_to_shift) & 0xFF


def as_ascii(byte):
    return "{0:0{1}x}".format(byte, 2)
