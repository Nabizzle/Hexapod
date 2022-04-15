from hexapod import ssc32uDriver
import numpy as np

def test_angleToPW():
    assert ssc32uDriver.angleToPW(90) == 1500

def test_anglesToSerial():
    angles = np.array([[0, 10, 20],
                        [30, 40, 50],
                        [60, 70, 80],
                        [90, 100, 110],
                        [120, 130, 140],
                        [150, 160, 170]])
    speed = 500
    time = 1000
    assert ssc32uDriver.anglesToSerial(angles, speed, time) == '#0P500S500' + '#1P611S500' + '#2P722S500' + '#3P833S500' + '#4P944S500' + '#5P1056S500' + '#6P1167S500' + '#7P1278S500' + '#8P1389S500' + '#9P1500S500' + '#10P1611S500' + '#11P1722S500' + '#12P1833S500' + '#13P1944S500' + '#14P2056S500' + '#15P2167S500' + '#16P2278S500' + '#17P2389S500' + 'T1000\r'
