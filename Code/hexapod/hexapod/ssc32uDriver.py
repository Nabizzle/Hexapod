import serial

def angleToPW(angle):
    #convert the input angle in degrees to the pulse width in us to command that angle
    return round(2000 * angle / 180 + 500) #returns the pulse width as the equivalent whole number between 500us (0 degrees) and 2500us (180 degrees)

def anglesToSerial(angles, speed, time):
    #converts an array of servo angles to the formatted serial command for the Lynxmotiohn SSC-32U
    if angles.shape == (6, 3):
        angles = angles.flatten()
    else:
        raise Exception('Input angles were the wrong format. Should be a 6x3 numpy array')
    #speed is in microseconds/second and time is in milliseconds. A speed of 1000us takes 1 second to go 90 degrees
    serial_string = ''
    for i, angle in enumerate(angles):
        serial_string += '#' + str(i) + 'P' + str(angleToPW(angle)) + 'S' + str(speed)

    serial_string += 'T' + str(time) + '\r'
    return serial_string

def connect(com):
    #tries to open a serial port with the Lynxmotiohn SSC-32U on the desired COM port
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = com
    ser.open()
    if ser.is_open:
        print(ser)
    else:
        print('Serial port did not open')
    return ser

def disconnect(ser):
    #disconnects for the serial port
    ser.close()
    if ser.is_open:
        print('Serial port is still open')
        return False

    print('Serial port is closed.')
    return True

def sendData(ser, serial_string):
    #sends the commands for the servos to the Lynxmotiohn SSC-32U
    ser.write(serial_string)
    return True
