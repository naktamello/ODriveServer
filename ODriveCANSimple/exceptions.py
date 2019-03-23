class ODriveCANInvalidNodeID(Exception):
    pass

class ODriveCANUnsupportedCommand(Exception):
    pass


class ODriveCANPacketException(Exception):
    pass

class ODriveCANSignatureMismatch(Exception):
    pass

class UartServerException(Exception):
    pass
