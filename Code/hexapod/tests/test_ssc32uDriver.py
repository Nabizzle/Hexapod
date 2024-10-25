from hexapod import ssc32uDriver
import numpy as np
import serial
from serial import SerialException
import pytest


def test_angleToPW():
    """Tests that the found pulse width for a 90 degree servo positions is 1500us"""
    assert ssc32uDriver.angleToPW(90) == 1500


def test_anglesToSerial():
    """Tests that the message to the servo controller is made and formatted correctly and as a byte string"""
    angles = np.array([[60, 60, 60],
                       [0, 60, 60],
                       [-60, 60, 60],
                       [-120, 60, 60],
                       [180, 60, 60],
                       [120, 60, 60]])
    speed = 500
    time = 1000
    assert ssc32uDriver.anglesToSerial(angles, speed, time) == bytes('#0P833S500' + '#1P833S500' + '#2P2167S500' + '#3P1500S500' + '#4P833S500' + '#5P2167S500' + '#6P2167S500' + '#7P833S500' + '#8P2167S500' + '#9P833S500' + '#10P2167S500' + '#11P833S500' + '#12P1500S500' + '#13P2167S500' + '#14P833S500' + '#15P2167S500' + '#16P2167S500' + '#17P833S500' + 'T1000\r', 'ascii')


def test_anglesToSerialWrongFormat():
    """Tests that an incorrectly formatted message raises an exception"""
    angles = np.array([[60, 60, 60],
                       [0, 60, 60],
                       [-60, 60, 60],
                       [-120, 60, 60],
                       [180, 60, 60]])
    speed = 500
    time = 1000
    with pytest.raises(Exception):
        ssc32uDriver.anglesToSerial(angles, speed, time)


def test_connect():
    """Tests that the connection to a serial port works"""
    test_ser = serial.serial_for_url('loop://?logging=debug')
    ser = ssc32uDriver.connect('loop://?logging=debug')
    assert ser.port == test_ser.port and ser.baudrate == 9600


def test_connectError():
    """Tests that error is raised when trying to connect without a servo driver"""
    with pytest.raises(SerialException):
        ssc32uDriver.connect('COM-1')


def test_disconnect():
    """Tests that closing port is successful"""
    ser = serial.serial_for_url('loop://?logging=debug')
    assert ssc32uDriver.disconnect(ser) is True


def test_sendData():
    """Tests sending data over a debug port"""
    ser = serial.serial_for_url('loop://?logging=debug')
    assert ssc32uDriver.sendData(ser, b'data')


def test_sendDataError():
    """Tests that sending data on a port that isn't connect causes an error"""
    ser = serial.Serial()
    with pytest.raises(SerialException):
        ssc32uDriver.sendData(ser, b'data')