import serial

portDD = '/dev/ttyUSB2'


serDD = serial.Serial(port=portDD,
                           baudrate=9600,
                           parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE,
                           bytesize=serial.EIGHTBITS,
                           timeout=1
                           )
serDD.isOpen()

while serDD:
    incoming = serDD.read()
    print (incoming)