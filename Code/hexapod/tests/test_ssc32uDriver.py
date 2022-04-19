from hexapod import ssc32uDriver
import numpy as np

def test_angleToPW():
    """Tests that the found pulse width for a 90 degree servo positions is 1500us"""
    assert ssc32uDriver.angleToPW(90) == 1500

def test_anglesToSerial():
    """Tests that the message to the servo controller is made and formatted correctly and as a btye string"""
    angles = np.array([[60, 60, 60],
                        [0, 60, 60],
                        [-60, 60, 60],
                        [-120, 60, 60],
                        [180, 60, 60],
                        [120, 60, 60]])
    speed = 500
    time = 1000
    assert ssc32uDriver.anglesToSerial(angles, speed, time) == bytes('#0P833S500' + '#1P833S500' + '#2P2167S500' + '#3P1500S500' + '#4P833S500' + '#5P2167S500' + '#6P2167S500' + '#7P833S500' + '#8P2167S500' + '#9P833S500' + '#10P2167S500' + '#11P833S500' + '#12P1500S500' + '#13P2167S500' + '#14P833S500' + '#15P2167S500' + '#16P2167S500' + '#17P833S500' + 'T1000\r', 'ascii')
