import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('localhost', 1978))
sock.settimeout(2)
from time import sleep

def send(cmd:str, recv=False):
    sock.send(cmd.encode())
    sleep(0.05)
    if recv:
        response = sock.recv(1024)
        print(response.decode())

def setup_joints():
    template = "robot:init_joint {}"
    send(template.format('j1'))
    send(template.format('j2'))
    send(template.format('j3'))
    send(template.format('j4'))
    send(template.format('j5'))
    send(template.format('j6'))
    response = sock.recv(1024)
    print(response.decode())

def energize_all():
    template = "can:{} state 8"
    send(template.format(1))
    send(template.format(2))
    send(template.format(3))
    send(template.format(4))
    send(template.format(5))
    send(template.format(6))

def deenergize_all():
    template = "can:{} state 1"
    send(template.format(1))
    send(template.format(2))
    send(template.format(3))
    send(template.format(4))
    send(template.format(5))
    send(template.format(6))

def move_p1():
    template = "can:{} setpos {}"
    send(template.format(1, -20000))
    send(template.format(2, -30000))
    send(template.format(3, -30000))
    send(template.format(4, 20000))
    send(template.format(5, 60000))
    send(template.format(6, 50000))

def move_p2():
    template = "can:{} setpos {}"
    send(template.format(1, 20000))
    send(template.format(2, -30000))
    send(template.format(3, -30000))
    send(template.format(4, -20000))
    send(template.format(5, 60000))
    send(template.format(6, 50000))

def shake():
    template = "can:{} setpos {}"
    send(template.format(4, 0))
    sleep(0.5)
    send(template.format(4, -20000))
    send(template.format(5, 30000))
    sleep(0.5)
    send(template.format(4, 20000))
    send(template.format(5, 30000))


def move_p0():
    template = "can:{} setpos {}"
    send(template.format(1, 0))
    send(template.format(2, 0))
    send(template.format(3, 0))
    send(template.format(4, 0))
    send(template.format(5, 0))
    send(template.format(6, 0))

def move():
    move_p1()
    sleep(0.6)
    move_p0()
    sleep(0.6)
    move_p2()
    sleep(0.6)
    move_p0()

# def move_p1():
#     j1.move_to(20000)
#     j2.move_to(15000)
#     j3.move_to(-30000)
#     j4.move_to(10000)
#     j5.move_to(-20000)
#     j6.move_to(50000)

