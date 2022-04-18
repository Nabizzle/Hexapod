from hexapod import rotation
import numpy as np
def test_xrot():
    """Tests that the x rotation matrix with a 90 degree rotation is correct"""
    assert np.allclose(rotation.xRot(90), np.array([[1, 0, 0],
                                                    [0, 0, 1],
                                                    [0, -1, 0]]))

def test_yRot():
    """Tests that the y rotation matrix with a 90 degree rotation is correct"""
    assert np.allclose(rotation.yRot(90), np.array([[0, 0, 1],
                                                    [0, 1, 0],
                                                    [-1, 0, 0]]))

def test_zRot():
    """Tests that the z rotation matrix with a 90 degree rotation is correct"""
    assert np.allclose(rotation.zRot(90), np.array([[0, 1, 0],
                                                    [-1, 0, 0],
                                                    [0, 0, 1]]))
