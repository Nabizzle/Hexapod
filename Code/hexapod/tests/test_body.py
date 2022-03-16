from hexapod import body
import numpy as np

def test_bodyPos():
    assert np.allclose(body.bodyPos(0, 0, 0, 0, 0, 0, 85), np.array([[42.5, 73.61215932, 0],
                                                                    [85, 0, 0],
                                                                    [42.5, -73.61215932, 0],
                                                                    [-42.5, -73.61215932, 0],
                                                                    [-85, 0, 0],
                                                                    [-42.5, 73.61215932, 0],
                                                                    [42.5, 73.61215932, 0]]))
