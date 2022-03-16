import serial

portDD = '/dev/ttyUSB2'

portBot = '/dev/ttyUSB0'


serDD = serial.Serial(port=portDD,
                           baudrate=9600,
                           parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE,
                           bytesize=serial.EIGHTBITS,
                           timeout=1
                           )

serBot = serial.Serial(port=portBot,
                           baudrate=9600,
                           parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE,
                           bytesize=serial.EIGHTBITS,
                           timeout=1
                           )

serDD.isOpen()
serBot.isOpen()

while serDD:
    incoming = serDD.read()
    print (f'DD         {incoming}')
