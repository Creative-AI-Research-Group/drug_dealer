#
# Drug Dealer Bot
# 10 August 2021
#
# Craig Vear - cvear@dmu.ac.uk
#

"""server script for receiving robot movement instructions
from Dmitriy's C++ CV script (localhost)"""

# import python modules
import sys
import serial
from time import sleep

# import project modules
from modules.rerobot import Robot
from modules.arm import Arm

# hardware abd logging
LOGGING = True
DD_HARDWARE = False
ARM = True

# consts
bot_stop = b'\x09' # 9 not 99
bot_forward = b'\x01' # 1
bot_backward = b'\x02' # 2
bot_left_turn = b'\x03' # 3
bot_right_turn = b'\x04' # 4

arm_open_claw = b'\x05' # 5
arm_close_claw = b'\x06' # 6

arm_waiting_pos = b'\x07' # 7
arm_get_pos = b'\x08' # 8

class Matlab:
    def __init__(self):
        if DD_HARDWARE:
            # port_name = input('what is the port of your CV hardware? e.g. ttyUSB1')

            # if sys.platform.startswith('win'):
            #     port = port_name
            # elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            #     # this excludes your current terminal "/dev/tty"
            #     port = '/dev/' + port_name
            # elif sys.platform.startswith('darwin'):
            #     port = '/dev/' + port_name
            # else:
            #     raise EnvironmentError('Unsupported platform')

            port = '/dev/ttyUSB2'

            if LOGGING:
                print(f'initialise connection to host\n'
                      f'opens up the serial port as an object called "ser"{port}')

            self.ser = serial.Serial(port=port,
                                     baudrate=9600,
                                     parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE,
                                     bytesize=serial.EIGHTBITS,
                                     timeout=1
                                     )
            self.ser.isOpen()

        # instantiate and own robot and LSS objects
        self.robot = Robot()
        self.robot.rotate(20)
        if LOGGING:
            print(f'Robot ready')

        if ARM:
            self.arm = Arm()
            self.arm.wait_ready()
            if LOGGING:
                print(f'arm ready')

    # listen to port
    # read from server buffer
    def read(self):
        if LOGGING:
            print('ready to read')

        if DD_HARDWARE:
            while self.ser.isOpen():
                # Read incoming SIP
                incoming = self.ser.read(255)

                if incoming == b'':
                    print('waiting for data')

                else:
                    data = incoming # int(incoming, 16)
                    if LOGGING:
                        print(f'READING = {incoming} = {data}')
                    self.parse_data(data)

                self.ser.flushInput()

        else:
            if ARM:
                demo = 8
            else:
                demo = 4

            for n in range(demo):
                self.parse_data(n+1)
                sleep(2)
                self.parse_data(9)

        self.terminate()

    # parses all data from
    def parse_data(self, data):
        print(f'parsing {data}')

        if data == bot_stop:
            self.robot.stop()
        elif data == bot_forward:
            # self.robot.forward(100)
            self.robot.step_forward()
        elif data == bot_backward:
            self.robot.backward()
        elif data == bot_left_turn:
            self.robot.rotate()
        elif data == bot_right_turn:
            self.robot.rotate(-10)

        elif data == arm_open_claw:
            self.arm.open_claw()
        elif data == arm_close_claw:
            self.arm.close_claw()

        elif data == arm_waiting_pos:
            self.arm.wait_ready()
        elif data == arm_get_pos:
            self.arm.arm_reach_out()

    def terminate(self):
        self.robot.terminate()

        if ARM:
            self.arm.terminate()

if __name__ == '__main__':
    dd_bot = Matlab()
    dd_bot.read()
