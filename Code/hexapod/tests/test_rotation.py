from hexapod import rotation
import numpy as np
def test_xrot():
    assert np.allclose(rotation.xRot(90), np.array([[1, 0, 0],
                                                    [0, 0, 1],
                                                    [0, -1, 0]]))

def test_yRot():
    assert np.allclose(rotation.yRot(90), np.array([[0, 0, 1],
                                                    [0, 1, 0],
                                                    [-1, 0, 0]]))

def test_zRot():
    assert np.allclose(rotation.zRot(90), np.array([[0, 1, 0],
                                                    [-1, 0, 0],
                                                    [0, 0, 1]]))
