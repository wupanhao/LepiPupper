#!coding:utf-8
import time

header = [0x12, 0x4c]
header = [0xff, 0xff]


def chk_sum(data):
    # data[-1] = 0
    # print(data)
    check = (~(sum(data[2:]) & 0xFF)) & 0xFF
    return check


class SERVO(object):
    PING = 1
    READ_DATA = 2
    WRITE_DATA = 3
    RESET = 6


class Servo:
    def __init__(self, id, position, ms=0, speed=0):
        self.id = id
        self.position = position
        self.ms = ms
        self.speed = speed

    def __repr__(self):
        return str({"id": self.id, "position": self.position})


class EEPROM(object):
    ID = 0x05
    MIN_POSITION_H = 0x09
    MIN_POSITION_L = 0x0a
    MAX_POSITION_H = 0x0b
    MAX_POSITION_L = 0x0c
    TARGET_POSITION_H = 0x2a
    TARGET_POSITION_L = 0x2b
    SPEED_H = 0x2e
    SPEED_L = 0x2f
    LOCK = 0x30
    CURRENT_POSITION_H = 0x38
    CURRENT_POSITION_L = 0x39


class SServo(object):
    """
    serial client
    """

    def __init__(self, port=None, baud_rate=1000000):
        import serial
        if port is None:
            import serial.tools.list_ports
            serial_ports = [i[0] for i in serial.tools.list_ports.comports()]
            print(serial_ports)
            if 'USB' in serial_ports[0]:
                port = serial_ports[0]
            else:
                port = '/dev/ttyAMA1'
        self.header = header
        self.port = serial.Serial(port=port, baudrate=baud_rate, bytesize=8, parity=serial.PARITY_NONE,
                                  stopbits=serial.STOPBITS_ONE, timeout=0.01)

    def send_cmd(self, cmd):
        self.port.write(cmd.encode('utf-8'))

    def read_cmd(self):
        response = self.port.readline()
        return response.decode('utf-8')

    def read_num(self, num):
        response = self.port.read(num)
        return response

    def send_hex(self, cmd):
        # print(cmd)
        self.port.write(cmd)
        time.sleep(0.00001)
        self.port.flushInput()

    def read_hex(self, n=0):
        if(n > 0):
            response = self.port.read(n)
        else:
            response = self.port.readline()
        array = []
        for i in response:
            array.append(ord(i))
        return array

    def ping(self, id):
        data = [header[0], header[1], id, 0x02, 0x01, 0]
        data[-1] = chk_sum(data)
        self.send_hex(data)
        res = self.read_hex(6)
        # print(res)
        if len(res) == 6 and res[2] == id:
            return True
        else:
            return False

    def reset(self, id):
        data = [header[0], header[1], id, 0x02, SERVO.RESET, 0]
        data[-1] = chk_sum(data)
        # print('transfer:',data)
        self.send_hex(data)
        return len(self.read_hex())

    def get_position(self, id):
        data = [header[0], header[1], id, 0x04, 0x02, 0x38, 0x02, 0]
        # data = [FF FF 01 09 03 2A 00 08 00 00 E8 03 D5]
        # data = [0xff,0xff ,0x01,0x09,0x03,0x2a,0x00,0x08,0x00,0x00,0xe8,0x03,0x00]
        # chk = chk_sum(data)
        data[-1] = chk_sum(data)
        # print('transfer:' ,''.join(format(x, '02x') for x in data))
        # print(data)
        self.send_hex(data)
        res = self.read_hex()
        if(len(res) == 8):
            return (res[-3] << 8) + res[-2]
        else:
            return 0

    def set_position(self, id, position, ms=0, speed=0):
        # 0 <= min_position <= position <= max_position <= 0x03ff (200 degree)
        # 0 <= min_speed <= speed <= max_speed <= 0x03ff
        if(position < 0):
            position = 0
        if(position > 0x03ff):
            position = 0x03ff
        if(speed < 0):
            speed = 0
        if(speed > 0x03ff):
            speed = 0x03ff
        if(ms < 0):
            ms = 0
        if(ms > 0x03ff):
            ms = 0x03ff
        data = [header[0], header[1], id, 0x09, 0x03, 0x2A, (position >> 8) & 0xFF, position & 0xFF, (
            ms >> 8) & 0xFF, ms & 0xFF, (speed >> 8) & 0xFF, speed & 0xFF, 0]
        data[-1] = chk_sum(data)
        # print(data)
        # print('transfer:', ''.join(format(x, '02x') for x in data))
        self.send_hex(data)
        return len(self.read_hex())

    def set_positions_sync(self, servos):
        num = len(servos)
        if not num > 0:
            return
        data = [header[0], header[1], 0XFE, num*7+4, 0X83, 0x2A, 0x06]
        for servo in servos:
            data.extend([servo.id, (servo.position >> 8) & 0xFF, servo.position & 0xFF, (
                servo.ms >> 8) & 0xFF, servo.ms & 0xFF, (servo.speed >> 8) & 0xFF, servo.speed & 0xFF])
        data.append(0)
        data[-1] = chk_sum(data)
        # print('transfer:', ''.join(format(x, '02x') for x in data))
        self.send_hex(data)
        return
        # return len(self.read_hex())

    def set_positions(self, servos):
        num = len(servos)
        if not num > 0:
            return
        for servo in servos:
            data = [header[0], header[1], servo.id, 9, 0X04, 0x2A, (servo.position >> 8) & 0xFF, servo.position & 0xFF, (
                servo.ms >> 8) & 0xFF, servo.ms & 0xFF, (servo.speed >> 8) & 0xFF, servo.speed & 0xFF, 0]
            # data.extend([servo.id,(servo.position >> 8) & 0xFF ,servo.position & 0xFF ,(servo.ms >> 8) & 0xFF ,servo.ms & 0xFF])
            data[-1] = chk_sum(data)
            print('transfer:', ''.join(format(x, '02x') for x in data))
            self.send_hex(data)
            print(self.read_hex())
        data = [header[0], header[1], 0XFE, 2, 0X05, 0x00]
        data[-1] = chk_sum(data)
        time.sleep(2)
        print('transfer:', ''.join(format(x, '02x') for x in data))
        self.send_hex(data)
        return len(self.read_hex())

    def set_servo_angles(self, angles):
        # angles = []
        ids = []
        for leg_index in range(4):
            for axis_index in range(3):
                id = leg_index*3+axis_index+1
                ids.append(id)
        servos = [Servo(ids[i], int(angles[i]/200.0*1023+1023/2), speed=2000)
                  for i in range(12)]
        self.set_positions_sync(servos)

    def scan(self):
        devices = []
        for i in range(253):
            if(self.ping(i)):
                devices.append(i)
                print(self.get_info(i))
        return devices

    def set_id(self, id_old, id_new):
        data = [header[0], header[1], id_old, 0x04,
                SERVO.WRITE_DATA, EEPROM.ID, id_new, 0]
        data[-1] = chk_sum(data)
        print('transfer:', ''.join(format(x, '02x') for x in data))
        self.send_hex(data)
        return len(self.read_hex())

    def write_u8(self, id, param, value):
        data = [header[0], header[1], id, 0x04,
                SERVO.WRITE_DATA, param, value, 0]
        data[-1] = chk_sum(data)
        print('transfer:', ''.join(format(x, '02x') for x in data))
        self.send_hex(data)
        return len(self.read_hex())

    def write_u16(self, id, param, value):
        data = [header[0], header[1], id, 0x05, SERVO.WRITE_DATA,
                param, (value >> 8) & 0xff, value & 0xff, 0]
        data[-1] = chk_sum(data)
        print('transfer:', ''.join(format(x, '02x') for x in data))
        self.send_hex(data)
        return len(self.read_hex())

    def read_u8(self, id, param):
        data = [header[0], header[1], id, 0x04,
                SERVO.READ_DATA, param, 0x01, 0]
        data[-1] = chk_sum(data)
        print('transfer:', ''.join(format(x, '02x') for x in data))
        self.send_hex(data)
        res = self.read_hex()
        if(len(res) == 7):
            return res[-2]
        else:
            return 0

    def read_u16(self, id, param):
        # print(id,param)
        data = [header[0], header[1], id, 0x04,
                SERVO.READ_DATA, param, 0x02, 0]
        data[-1] = chk_sum(data)
        # print('transfer:' ,''.join(format(x, '02x') for x in data))
        self.send_hex(data)
        res = self.read_hex()
        if(len(res) == 8):
            return (res[-3] << 8) + res[-2]
        else:
            return 0

    def get_info(self, id):
        info = [0, 0, 0]
        info[0] = self.read_u16(id, EEPROM.MIN_POSITION_H)
        info[1] = self.read_u16(id, EEPROM.CURRENT_POSITION_H)
        info[2] = self.read_u16(id, EEPROM.MAX_POSITION_H)
        return info

    # Change the id forever
    def change_id(self, old, new):
        # print(self.read_u8(1, EEPROM.LOCK))
        self.write_u8(old, EEPROM.LOCK, 0)
        time.sleep(0.1)
        self.set_id(old, new)
        time.sleep(0.1)
        self.write_u8(new, EEPROM.LOCK, 1)
        time.sleep(0.2)
        return self.ping(new)

    def scan_print(self, n=254):
        servos = []
        print("正在扫描,包头: "),
        print("0x%X 0x%X" % (self.header[0], self.header[1]))
        print("检测到舵机:"),
        for i in range(n):
            if self.ping(i):
                servos.append(i)
                print(str(i)+' '),
                self.set_position(i, 1023/2, ms=500)
        print("\n扫描结束")
        return servos


if __name__ == '__main__':
    import serial.tools.list_ports
    serial_ports = [i[0] for i in serial.tools.list_ports.comports()]
    print(serial_ports)
    # servo = SServo('/dev/ttyAMA1')
    servo = SServo()
    # servo.send_hex([header[0],header[1], 0x01, 0x02, 0x01, 0xFB])
    # print(servo.read_hex())
    # while True:
    #     print(servo.read_hex())
    #     time.sleep(0.1)
    # exit(0)

    # print(servo.read_u8(1, EEPROM.LOCK))
    '''
    # Change the id forever
    def change
    print(servo.read_u8(1, EEPROM.LOCK))
    servo.write_u8(1, EEPROM.LOCK, 0)
    time.sleep(0.1)
    servo.set_id(1, 2)
    time.sleep(0.1)
    servo.write_u8(2, EEPROM.LOCK, 1)
    '''
    servo.scan_print(25)
    # exit(0)
    '''
    # print(i, servo.ping(i))
    # print(servo.get_info(i))
    # exit(0)
    test_data = [header[0],header[1], 0XFE, 0X18, 0X83, 0X38, 0X04, 0X00, 0X00, 0X10, 0X03, 0XE8, 0X01, 0X02, 0X20,
        0X03, 0XE8, 0X02, 0X00, 0X30, 0X03, 0XE8, 0X03, 0X02, 0X20, 0X03, 0XE8, 0X02]
    print('transfer:' ,''.join(format(x, '02x') for x in test_data))
    servo.send_hex(test_data)
    print(servo.read_hex())
    exit(0)
    '''
    while True:
        start = time.time()
        servo.set_positions_sync(
            [Servo(1, 1023/2, speed=2000), Servo(2, 1023/2, speed=1000)])
        # print('transort costs %f ms' % ((time.time() - start)*1000))
        time.sleep(1.5)
        # servo.set_positions_sync(
        #     [Servo(1, 600, speed=2000), Servo(2, 650, speed=1000)])
        # time.sleep(1.5)
        # break
    exit(0)
    while True:
        servo.set_position(1, 450, ms=1000)
        servo.set_position(2, 450, ms=1000)
        time.sleep(1.5)
        servo.set_position(1, 1450, ms=1000)
        servo.set_position(2, 1450, ms=1000)
        time.sleep(1.5)
        # print(servo.ping(2))
        # print(servo.ping(3))
