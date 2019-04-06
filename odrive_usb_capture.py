import odrive
from time import sleep
from serial import Serial

uart = Serial('/dev/ttyJTCTRL', 115200)

print('looking for odrive12...')
odrv12 = odrive.find_any(serial_number="366333693037")
print('looking for odrive34...')
odrv34 = odrive.find_any(serial_number="208037893548")
print('looking for odrive56...')
odrv56 = odrive.find_any(serial_number="208037713548")
print('found!')


def run_until_failure():
    try:
        while True:
            count12 = odrv12.axis0.encoder.count_in_cpr
            count34 = odrv34.axis0.encoder.count_in_cpr
            count56 = odrv56.axis0.encoder.count_in_cpr
            # print(count12, count34, count56)
    except Exception as e:
        print(e)
        uart.write('a'.encode())
