import serial
import numpy as np
from numpy.typing import NDArray
from typing import Any, Optional


def angleToPW(angle: float) -> float:
    """
    convert the input angle in degrees to the pulse width in us to command
    that angle
    """
    # returns the pulse width as the equivalent whole number between 500us
    # (0 degrees) and 2500us (180 degrees)
    return round(2000 * angle / 180 + 500)


def anglesToSerial(angles: NDArray, speed: Optional[int] = None,
                   time: Optional[int] = None) -> bytes:
    """
    converts an array of servo angles to the formatted serial command for
    the Lynxmotiohn SSC-32U
    """
    if angles.shape == (6, 3):
        temp_angles = angles
        temp_angles[0:3, 2] = - temp_angles[0:3, 2]
        temp_angles[3:6, 1] = - temp_angles[3:6, 1]
        adjustment = np.array([[90, 90, 90],
                               [90, 90, 90],
                               [90, 90, 90],
                               [-90, 90, 90],
                               [270, 90, 90],
                               [270, 90, 90]])
        temp_angles = adjustment - temp_angles
        angles_processed = temp_angles.flatten()
    else:
        raise Exception(
            'Input angles were the wrong format. Should be a 6x3 numpy array')
    # speed is in microseconds/second and time is in milliseconds. A speed of
    # 1000us takes 1 second to go 90 degrees
    serial_string = ''
    for i, angle in enumerate(angles_processed):
        serial_string += '#' + str(i) + 'P' + \
            str(angleToPW(angle))
        if speed is not None:
            serial_string += 'S' + str(speed)

    if time is not None:
        serial_string += 'T' + str(time)
    serial_string += '\r'
    return bytes(serial_string, 'ascii')


def connect(com: str) -> Any:
    """
    tries to open a serial port with the Lynxmotiohn SSC-32U on the desired
    COM port
    """
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = com
    ser.open()
    if ser.is_open:
        print(ser)
    else:
        print('Serial port did not open')
    return ser


def disconnect(ser: Any) -> bool:
    """disconnects for the serial port"""
    ser.close()
    if ser.is_open:
        print('Serial port is still open')
        return False

    print('Serial port is closed.')
    return True


def sendData(ser: Any, serial_string: bytes) -> bool:
    """sends the commands for the servos to the Lynxmotiohn SSC-32U"""
    ser.write(serial_string)
    return True
