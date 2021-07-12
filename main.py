#
# Drug Dealer Bot
# 19 July 2021
#
# Craig Vear - cvear@dmu.ac.uk
#

"""server script for receiving robot movement instructions
from Dmitriy's C++ CV script (localhost)"""

import socket
from ReRoROS.rerobot import Robot
from lynxmotion_LSS_drawing_arm.arm import Arm

class BotServer:

    def __init__(self):
        # define socket params
        self.HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
        self.PORT = 65432
        self.connected = False

        # instantiate and own robot and LSS objects
        self.robot = Robot()
        self.arm = Arm()


    # parses all data from
    def parse_data(self, data):
        pass

    # opens socket and listens to Dmitriy's instruction to move robot
    def server(self):
        # init socket stream and wait for data
        print("client: started!")
        print(f"client: connecting to {self.HOST}:{self.PORT}")

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.HOST, self.PORT))
            s.listen()
            client_stream, addr = s.accept()
            with client_stream:
                print('Connected by', addr)
                self.connected = True
                while self.connected:
                    # get data from stream
                    data = client_stream.recv(1024)
                    print(f"receiver: got data {data}")

                    # send it back for double check
                    # todo checksum
                    client_stream.sendall(data)

                    # parse data
                    self.parse_data(data)

                # closing down sequence
                print('Closing down, baby')
                client_stream.close()
                self.terminate()

    def terminate(self):
        self.robot.terminate()
        self.arm.terminate()


if __name__ == '__main__':
    bot = BotServer()
    bot.server()