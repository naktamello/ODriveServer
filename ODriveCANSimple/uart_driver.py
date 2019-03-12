from typing import Optional
import sys
from serial import Serial
import asyncio
import serial_asyncio

q = asyncio.Queue()


def got_stdin_data(queue):
    asyncio.async(queue.put(sys.stdin.readline()))




class AsyncUart(asyncio.Protocol):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.buffer = []

    def connection_made(self, transport: serial_asyncio.SerialTransport):
        self.transport = transport
        print('port opened', transport)
        self.transport.serial.rts = False
        fut = asyncio.async(q.get())
        fut.add_done_callback(self.write_message)

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

    def write_message(self, fut):
        reply = fut.result()
        print('Sending: {!r}'.format(reply))
        self.transport.write(reply.encode())
        fut = asyncio.async(q.get())
        fut.add_done_callback(self.write_message)


if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.add_reader(sys.stdin, got_stdin_data, q)
    coroutine0 = serial_asyncio.create_serial_connection(loop, AsyncUart, '/dev/ttyJ1', 115200)
    loop.run_until_complete(coroutine0)
    loop.run_forever()
