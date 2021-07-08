#!coding:utf-8
from sservo import SServo
import time
import os
import sys
import tty
import termios


def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    # print(ch, ord(ch))
    return ord(ch)


def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if c1 == 0x03:
        exit()
    if c1 != 0x1b:
        return c1
    c2 = getchar()
    c3 = getchar()
    return c2*100+c3
    # sys.stdin.flush()


class KEY:
    ArrowLeft = 9168
    ArrowUp = 9165
    ArrowRight = 9167
    ArrowDown = 9166
    Enter = 13
    Memu = 7980  # KeyM (Menu)
    Back = 7981  # KeyB (Back)
    Run = 7982  # KeyR (Run)
    Stop = 27  # KeyS (Stop or Home)


'''
Enter 13
Up 9165
Down 9166 
Left 9168 
Right 9167
F1 7980
F2 7981
F3 7982
'''


def test():
    while True:
        c = readkey()
        if c == 3:  # ord ctrl-C
            break
        print(c)
    exit()


if __name__ == '__main__':
    import serial.tools.list_ports
    serial_ports = [i[0] for i in serial.tools.list_ports.comports()]
    print(serial_ports)
    # servo = SServo('/dev/ttyAMA1')
    servo = SServo()
    n = 1
    while True:
        os.system('clear')
        print("准备编号%d号舵机\n确认键继续,左右键修改id,返回键结束" % n)
        c = readkey()
        if c == KEY.ArrowRight:
            n = n+1
            continue
        elif c == KEY.ArrowLeft and n > 1:
            n = n-1
            continue
        elif c == KEY.Back:
            break
        elif c == KEY.Enter:
            servos = servo.scan_print(25)
            if len(servos) == 1:
                if servos[0] == n:
                    print('编号正确')
                else:
                    if servo.change_id(servos[0], n):
                        print('编号成功')
                    else:
                        print('编号失败')
                os.system('clear')
                servo.scan_print(25)
            else:
                print('舵机数量不为1:', len(servos))
        print("确认键继续,返回键结束")
        if readkey() == KEY.Back:
            break
