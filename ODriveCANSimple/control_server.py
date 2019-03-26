from asyncio import Future
from dataclasses import dataclass
from typing import Optional, Tuple, Union, List
import sys
import ODriveCANSimple.enums as enums
import asyncio
import serial_asyncio
from time import sleep
from ODriveCANSimple.can_interface import ODriveCANInterface
from ODriveCANSimple.exceptions import UartServerException
from ODriveCANSimple.helper import valid_amt_angle
from ODriveCANSimple.robot import RoboticArm, Joint

robotic_arm = RoboticArm()
cmd_queue = asyncio.Queue(maxsize=32)
tcp_queue = asyncio.Queue(maxsize=32)
rsp_queue = asyncio.Queue(maxsize=32)

critical = []  # TODO timeout invalidation


@dataclass
class ResponseFilter:
    node_id: int
    cmd_id: int
    callback: any


@dataclass
class CANResponse:
    node_id: int
    cmd_id: int
    data: List[any]


response_filters = []  # type: List[ResponseFilter]
response_processors = dict()
response_processors[enums.MSG_ODRIVE_HEARTBEAT] = 'update_heartbeat'
response_processors[enums.MSG_GET_ENCODER_COUNT] = 'update_encoder_count'
response_processors[enums.MSG_GET_ENCODER_OFFSET] = 'update_encoder_offset'


class InitializeJoint:
    def __init__(self, joint: Joint):
        self.joint = joint
        self.step_idx = 0
        self.steps = 2  # TODO count method names starting with step by introspection

    async def step0(self, *args):
        critical.append(True)
        self.joint.reset_state()
        if self.joint.error > 0:
            await tcp_queue.put(f"{self.joint.verbose_name} has error")
            return
        if not valid_amt_angle(self.joint.motor_angle):
            await tcp_queue.put("invalid amt angle")
            return
        self.joint.calculate_offset()
        offset = self.joint.offset.setpoint
        can_node_id = self.joint.config.can_node_id
        await cmd_queue.put(f"{can_node_id} woffset {offset}")
        response_filters.append(ResponseFilter(can_node_id, enums.MSG_GET_ENCODER_OFFSET, self))
        await cmd_queue.put(f"{can_node_id} roffset")

    async def step1(self, *args):
        can_node_id = self.joint.config.can_node_id
        can_response, *_ = args
        print('on step 1', can_response)
        if len(critical):
            critical.pop()

    async def __call__(self, can_response=None):
        method = getattr(self, f"step{self.step_idx}")
        self.step_idx += 1
        await method(can_response)

    def __repr__(self):
        return f"{self.__class__.__name__}:step{self.step_idx}"


class RobotAPI:
    def __init__(self):
        self.pending = []

    async def run(self, command: str):
        try:
            method, *tokens = command.split(" ")
            await getattr(self, method)(*tokens)
        except Exception as e:
            print('RobotAPI exception occured', e)

    async def get_zero(self, *args):
        joint_name, = args
        joint = robotic_arm.joint(joint_name)
        await asyncio.ensure_future(tcp_queue.put(str(joint.get_zero_position()) + "\n"))

    async def init_joint(self, *args):
        joint_name, = args
        joint = robotic_arm.joint(joint_name)
        robot_command = InitializeJoint(joint)
        await robot_command()

    async def get_target(self, *args):
        joint_name, angle = args
        joint = robotic_arm.joint(joint_name)
        target = joint.get_target_position(int(angle))
        await asyncio.ensure_future(tcp_queue.put(str(target) + "\n"))


robot_api = RobotAPI()


def update_heartbeat(response: CANResponse):
    joint = robotic_arm.search_by_can_node(response.node_id)
    error, state = response.data
    joint.requested_state.actual = state
    joint.error = error


def update_encoder_count(response: CANResponse):
    joint = robotic_arm.search_by_can_node(response.node_id)
    shadow, cpr = response.data
    joint.cpr = cpr
    joint.shadow_count = shadow
    print('updated encoder count', str(joint))


def update_encoder_offset(response: CANResponse):
    joint = robotic_arm.search_by_can_node(response.node_id)
    offset, is_ready = response.data
    joint.encoder_is_ready.actual = is_ready
    joint.offset.actual = offset
    print('updated encoder offset', str(joint))


def process_stdin_data(queue):
    asyncio.ensure_future(queue.put(sys.stdin.readline()))


async def get_heartbeat():
    await asyncio.sleep(1)
    while True:
        if len(critical) > 0:
            await asyncio.sleep(0.5)
        else:
            for joint in robotic_arm.joints:
                if len(critical) > 0:
                    continue
                command = "{} heartbeat".format(joint.config.can_node_id)
                await asyncio.ensure_future(cmd_queue.put(command))
                await asyncio.sleep(0.1)


def check_response_filter(response: CANResponse):
    needle = f"{response.node_id}:{response.cmd_id}"
    haystack = [f"{f.node_id}:{f.cmd_id}" for f in response_filters]
    if needle not in haystack:
        return
    idx = haystack.index(needle)
    return response_filters.pop(idx)


async def process_response():
    while True:
        response = await asyncio.ensure_future(rsp_queue.get())  # type: CANResponse
        match = check_response_filter(response)
        if match:
            await match.callback(response)
        elif response.cmd_id in response_processors:
            func_name = response_processors[response.cmd_id]
            globals()[func_name](response)


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
        elif message.startswith('robot:'):
            message = message.strip('robot:')
            asyncio.ensure_future(robot_api.run(message))
        elif 'break' in message:
            print('breakpoint')
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


class EncoderUartServer(asyncio.Protocol):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.buffer = []

    def connection_made(self, transport: serial_asyncio.SerialTransport):
        self.transport = transport
        print('EncoderUartServer serial port opened')
        self.transport.serial.rts = False

    def connection_lost(self, exc: Optional[Exception]):
        print('port closed')
        self.transport.loop.stop()

    def data_received(self, data: bytes):
        self.buffer.append(data.decode())
        contents = "".join(self.buffer)
        if contents.count('\n') > 1:
            self.buffer = []
            # raise UartServerException("command not processed in time")
        elif '\n' in contents:
            to_process, rest = contents.split("\n")
            self.buffer = [rest] if rest != "" else []
            name, angle1, angle2 = self.parse_message(to_process)
            self.buffer = []
            if not name:
                return
            joint = robotic_arm.joint(name)
            joint.motor_angle = angle1
            joint.output_angle = angle2

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
        contents = "".join(self.buffer)
        if contents.count('\r') > 1:
            self.buffer = []
            # raise UartServerException("command not processed in time")
        elif '\r' in contents:
            try:
                to_process, rest = contents.split("\r")
                self.buffer = [rest] if rest != "" else []
                node_id, cmd_id, values = self.interface.process_response(to_process)
                asyncio.ensure_future(rsp_queue.put(CANResponse(node_id, cmd_id, values)))
                if cmd_id != enums.MSG_ODRIVE_HEARTBEAT:
                    asyncio.ensure_future(tcp_queue.put(str(values) + "\n"))
                    print("CANUartServer raw:", to_process)
                    print("parsed: node={}, cmd_id={}, values=".format(node_id, cmd_id), values)
            except Exception as e:
                print('CANUartServer data_received exception', data.decode(), e)
                self.buffer = []

    def process_user_input(self, fut):
        command_raw = fut.result().strip('\n')
        tokens = command_raw.split(' ')
        if tokens[-1] != 'heartbeat':
            print('Received: {!r}'.format(command_raw))
        try:
            packet_ascii = self.interface.process_command(tokens)
            if tokens[-1] != 'heartbeat':
                print('Sending: {!r}'.format(packet_ascii))
            self.transport.write(packet_ascii.encode())
        except Exception:
            print("INVALID")
        fut = asyncio.ensure_future(cmd_queue.get())
        fut.add_done_callback(self.process_user_input)


async def divine_intervention():
    await asyncio.sleep(5)
    # robot_command = InitializeJoint(robotic_arm.joint('j3'))
    # robot_command()


if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.add_reader(sys.stdin, process_stdin_data, cmd_queue)
    coroutine0 = serial_asyncio.create_serial_connection(loop, CANUartServer, '/dev/tty232-0', 115200)
    loop.run_until_complete(coroutine0)
    coroutine1 = serial_asyncio.create_serial_connection(loop, EncoderUartServer, '/dev/ttyJ1', 115200)
    loop.run_until_complete(coroutine1)
    coroutine2 = loop.create_server(IOServer, '127.0.0.1', 1978)
    server = loop.run_until_complete(coroutine2)
    coroutine3 = loop.create_task(get_heartbeat())
    coroutine4 = loop.create_task(process_response())
    coroutine5 = loop.create_task(divine_intervention())
    loop.run_until_complete(asyncio.gather(coroutine3, coroutine4, coroutine5))
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass

    # Close the server
    server.close()
    loop.run_until_complete(server.wait_closed())
    loop.close()
