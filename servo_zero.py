#!coding:utf-8
from sservo import SServo
import time
import os

if __name__ == '__main__':
    import serial.tools.list_ports
    serial_ports = [i[0] for i in serial.tools.list_ports.comports()]
    print(serial_ports)
    # servo = SServo('/dev/ttyAMA1')
    servo = SServo()
    while True:
        os.system('clear')
        servo.scan_print(25)
        time.sleep(1)
