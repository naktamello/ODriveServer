from asyncio import Future
from typing import Optional, Tuple, Union
import sys
from serial import Serial
import asyncio
import serial_asyncio

from ODriveCANSimple.can_interface import ODriveCANInterface

cmd_queue = asyncio.Queue()
tcp_queue = asyncio.Queue()


def process_stdin_data(queue):
    asyncio.ensure_future(queue.put(sys.stdin.readline()))


class IOServer(asyncio.Protocol):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.fut = None

    def connection_made(self, transport: asyncio.transports.Transport):
        peername = transport.get_extra_info('peername')
        print('TCP connection from {}'.format(peername))
        self.transport = transport
        self.setup_callback()

    def setup_callback(self):
        self.fut = asyncio.ensure_future(tcp_queue.get())
        self.fut.add_done_callback(self.handle_response)

    def connection_lost(self, exc):
        peername = self.transport.get_extra_info('peername')
        self.fut.cancel()
        print('connection lost:{}'.format(peername))

    def data_received(self, data):
        try:
            message = data.decode().strip("\r\n")
            self.handle_remote_request(message)
        except Exception:
            pass
        # fut = asyncio.ensure_future(q.get())
        # fut.add_done_callback(self.write_reply)

    def handle_remote_request(self, message: str):
        # TODO client needs to send a request ID so response can be matched
        if message.startswith('can:'):
            command = message.split('can:')[-1]
            print('processing command: {!r}'.format(command))
            asyncio.ensure_future(cmd_queue.put(command))
        elif message.startswith('check_angle:'):
            message = message.strip('check_angle:')

        else:
            print('unhandled request: {!r}'.format(message))

    def handle_response(self, fut: Future):
        peername = self.transport.get_extra_info('peername')
        print('sending to:{}'.format(peername))
        if fut.cancelled():
            return
        response = fut.result()
        self.transport.write(response.encode())
        self.setup_callback()


class AMTUartServer(asyncio.Protocol):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.buffer = []

    def connection_made(self, transport: serial_asyncio.SerialTransport):
        self.transport = transport
        print('CANUartServer serial port opened')
        self.transport.serial.rts = False

    def connection_lost(self, exc: Optional[Exception]):
        print('port closed')
        self.transport.loop.stop()

    def data_received(self, data: bytes):
        self.buffer.append(data.decode())
        if b'\n' in data:
            print("AMTUartServer raw:", "".join(self.buffer).strip('\n'))
            self.buffer = []

    @staticmethod
    def parse_message(msg) -> Union[Tuple[str, int, int], Tuple[None, None, None]]:
        try:
            name, angles = msg.strip().split(":")
            angle1, angle2 = angles.split("/")
            return name, int(angle1), int(angle2)
        except Exception:
            return None, None, None


class CANUartServer(asyncio.Protocol):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.buffer = []
        self.interface = ODriveCANInterface()

    def connection_made(self, transport: serial_asyncio.SerialTransport):
        self.transport = transport
        print('CANUartServer serial port opened')
        self.transport.serial.rts = False
        fut = asyncio.ensure_future(cmd_queue.get())
        fut.add_done_callback(self.process_user_input)

    def connection_lost(self, exc: Optional[Exception]):
        print('port closed')
        self.transport.loop.stop()

    def data_received(self, data: bytes):
        self.buffer.append(data.decode())
        if b'\r' in data:
            print("CANUartServer raw:", "".join(self.buffer))
            values = self.interface.process_response("".join(self.buffer))
            asyncio.ensure_future(tcp_queue.put(str(values)))
            self.transport.write(str(values).encode())
            print("parsed:", values)
            self.buffer = []

    def process_user_input(self, fut):
        command_raw = fut.result().strip('\n')
        tokens = command_raw.split(' ')
        print('Received: {!r}'.format(command_raw))
        try:
            packet_ascii = self.interface.process_command(tokens)
            print('Sending: {!r}'.format(packet_ascii))
            self.transport.write(packet_ascii.encode())
        except Exception:
            print("INVALID")
        fut = asyncio.ensure_future(cmd_queue.get())
        fut.add_done_callback(self.process_user_input)


if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.add_reader(sys.stdin, process_stdin_data, cmd_queue)
    coroutine0 = serial_asyncio.create_serial_connection(loop, CANUartServer, '/dev/tty232-0', 115200)
    loop.run_until_complete(coroutine0)
    coroutine1 = serial_asyncio.create_serial_connection(loop, AMTUartServer, '/dev/ttyJ2', 115200)
    loop.run_until_complete(coroutine1)
    coroutine2 = loop.create_server(IOServer, '127.0.0.1', 1978)
    server = loop.run_until_complete(coroutine2)
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass

    # Close the server
    server.close()
    loop.run_until_complete(server.wait_closed())
    loop.close()
