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
import atexit
import threading
import queue

# import project modules
from modules.rerobot import Robot
from modules.arm import Arm

# hardware abd logging
LOGGING = True
DD_HARDWARE = True
ARM = True

# consts
bot_stop = 0  # b'\x09' # 9 not 99
bot_forward = 1  # b'\x01' # 1
bot_backward = 2  # b'\x02' # 2
bot_left_turn = 3  # b'\x03' # 3
bot_right_turn = 4  # b'\x04' # 4

arm_open_claw = 5  # b'\x05' # 5
arm_close_claw = 6  # b'\x06' # 6

arm_waiting_pos = 7  # b'\x07' # 7
arm_get_pos = 8 #  b'\x08' # 8

movement_list = ["bot_forward",
                 "bot_backward",
                 "bot_left_turn",
                 "bot_right_turn",
                 "arm_open_claw",
                 "arm_close_claw",
                 "arm_waiting_pos",
                 "arm_get_pos"]

class DD_signal_in:
    def __init__(self):
        # instantiate and own robot and LSS objects
        self.robot = Robot()

        # move to ackowledge connection
        self.robot.rotate(20)
        if LOGGING:
            print(f'Robot ready')

        if ARM:
            self.arm = Arm()

            # move to ackowledge connection
            self.arm.wait_ready()
            if LOGGING:
                print(f'arm ready')

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

            self.portDD = '/dev/ttyUSB2'

            if LOGGING:
                print(f'initialise connection to host\n'
                      f'opens up the serial port to DD hardware as an object called "ser"{self.portDD}')

            self.serDD = serial.Serial(port=self.portDD,
                                       baudrate=9600,
                                       parity=serial.PARITY_NONE,
                                       stopbits=serial.STOPBITS_ONE,
                                       bytesize=serial.EIGHTBITS,
                                       timeout=1
                                       )
            self.serDD.isOpen()
            # atexit.register(self.terminate())

    # read from server buffer
    def read(self, inputQueue):
        while self.serDD.isOpen():

            # Check if incoming bytes are waiting to be read from the serial input
            # buffer.
            # NB: for PySerial v3.0 or later, use property `in_waiting` instead of
            # function `inWaiting()` below!
            if self.serDD.inWaiting() > 0:
                # read the bytes and convert from binary array to ASCII
                incoming = self.serDD.read(self.serDD.inWaiting()) #.decode('ascii')
                print('incoming data =  ', incoming)
                # print the incoming string without putting a new-line
                # ('\n') automatically after every print()

                # incoming = incoming[0]
                print('incoming data =  ', incoming)
                inputQueue.put(incoming)

                sleep(0.01)

    # listen to port
    def main(self):
        # self.serDD.flushInput()
        if LOGGING:
            print('ready to read')

        if DD_HARDWARE:
            EXIT_COMMAND = "exit"
            inputQueue = queue.Queue()

            inputThread = threading.Thread(target=self.read, args=(inputQueue,), daemon=True)
            inputThread.start()

            while self.serDD.isOpen():
                if inputQueue.qsize() > 0:
                    input_str = inputQueue.get()
                    print("input_str = {}".format(input_str))

                    if input_str == EXIT_COMMAND:
                        print("Exiting serial terminal.")
                        break

                    # Insert your code here to do whatever you want with the input_str.

                    # The rest of your program goes here.
                    # data = input_str[0]  # int(incoming, 16)
                    if LOGGING:
                        print(f'READING: {input_str} from {self.portDD}')
                    self.parse_data(input_str)

                    # sleep(0.01)
            print("End.")
        # if DD_HARDWARE:
        #     while self.serDD.isOpen():
        #
        #         # Check if incoming bytes are waiting to be read from the serial input
        #         # buffer.
        #         # NB: for PySerial v3.0 or later, use property `in_waiting` instead of
        #         # function `inWaiting()` below!
        #         if self.serDD.inWaiting() > 0:
        #             # read the bytes and convert from binary array to ASCII
        #             incoming = self.serDD.read(255)
        #             # print the incoming string without putting a new-line
        #             # ('\n') automatically after every print()
        #             print(incoming, end='')

                    # Put the rest of your code you want here
                # else:
                #     data = incoming[0]  # int(incoming, 16)
                #     if LOGGING:
                #         print(f'READING: {incoming} = {data} from {self.portDD}')
                #     self.parse_data(data)

                # self.serDD.flushInput()

                # Optional, but recommended: sleep 10 ms (0.01 sec) once per loop to let
                # other threads on your PC run during this time.

                # sleep(0.01)



                # # Read incoming SIP
                # incoming = self.serDD.read() # (255)
                #
                # if incoming == b'':
                #     print('waiting for data')



        # else:
        #     self.demo()

        self.terminate()

    def demo(self):
        if ARM:
            demo_list = 8
        else:
            demo_list = 4

        for n in range(demo_list):
            print(f'DEMO - testing function {movement_list[n]}')
            self.parse_data(n+1)
            sleep(2)
            self.parse_data(0)

    # parses all data from
    def parse_data(self, data):
        print(f'parsing {data}')

        try:
            if data == 0: # bot_stop:
                self.robot.stop()

            elif data == 1: # bot_forward:
                self.robot.step_forward()

            elif data == 2: # bot_backward:
                self.robot.step_backward()

            elif data == 3: # bot_left_turn:
                self.robot.step_left()

            elif data == 4: # bot_right_turn:
                self.robot.step_right()

            elif data == 5: # arm_open_claw:
                self.arm.open_claw()

            elif data == 6: # arm_close_claw:
                self.arm.close_claw()

            elif data == 7: # arm_waiting_pos:
                self.arm.wait_ready()

            elif data == 8: # arm_get_pos:
                self.arm.arm_reach_out()

            else:
                print("Dummy data")

        except:
            print('data corrupt')

        # if DD_HARDWARE:
        #     self.send_rx()

    # def send_rx(self):
    #     msg_hx = b'\x09'
    #     print(f'sending hex message: {msg_hx} to {self.serDD.port}')
    #     self.serDD.write(msg_hx)

    def terminate(self):
        self.robot.terminate()

        if ARM:
            self.arm.terminate()

        self.serDD.close()

if __name__ == '__main__':
    dd_bot = DD_signal_in()
    # dd_bot.demo()
    dd_bot.main()
