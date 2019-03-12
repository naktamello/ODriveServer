from typing import Optional
import sys
from serial import Serial
import asyncio
import serial_asyncio

from ODriveCANSimple.can_interface import ODriveCANInterface

q = asyncio.Queue()


def process_stdin_data(queue):
    asyncio.async(queue.put(sys.stdin.readline()))



class AsyncUart(asyncio.Protocol):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.buffer = []
        self.interface = ODriveCANInterface()

    def connection_made(self, transport: serial_asyncio.SerialTransport):
        self.transport = transport
        print('port opened', transport)
        self.transport.serial.rts = False
        fut = asyncio.async(q.get())
        fut.add_done_callback(self.process_user_input)

    def data_received(self, data: bytes):
        # print('AsyncUart:', repr(data))
        self.buffer.append(data.decode())
        if b'\r' in data:
            print("data:", "".join(self.buffer))
            self.transport.write("".join(self.buffer).encode())
            self.buffer = []

    def connection_lost(self, exc: Optional[Exception]):
        print('port closed')
        self.transport.loop.stop()

    def pause_writing(self):
        print('pause writing')
        print(self.transport.get_write_buffer_size())

    def resume_writing(self):
        print(self.transport.get_write_buffer_size())
        print('resume writing')

    def process_user_input(self, fut):
        command_raw = fut.result().strip('\n')
        tokens = command_raw.split(' ')
        print('Received: {!r}'.format(command_raw))
        packet_ascii = self.interface.process_command(tokens)
        print('Sending: {!r}'.format(packet_ascii))
        self.transport.write(packet_ascii.encode())
        fut = asyncio.async(q.get())
        fut.add_done_callback(self.process_user_input)


if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.add_reader(sys.stdin, process_stdin_data, q)
    coroutine0 = serial_asyncio.create_serial_connection(loop, AsyncUart, '/dev/ttyUSB1', 115200)
    loop.run_until_complete(coroutine0)
    loop.run_forever()
