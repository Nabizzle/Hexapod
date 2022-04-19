import numpy as np
from numpy.linalg import inv
from numpy.typing import NDArray
from hexapod.rotation import xRot, yRot, zRot
from math import sin, pi


def bodyPos(pitch: float = 0, roll: float = 0, yaw: float = 0, Tx: float = 0,
            Ty: float = 0, Tz: float = 0, body_offset: float = 85) -> NDArray:
    """
    Applies rotations in pitch, roll, and yaw and translations in x, y, and
    z to the body.
    """
    body_rot = np.matmul(zRot(yaw), yRot(roll))
    body_rot = np.matmul(body_rot, xRot(pitch))

    body_0 = np.matmul(inv(body_rot), np.array([[body_offset / 2,
                                                 body_offset * sin(pi / 3),
                                                 0]]).T)
    body_1 = np.matmul(inv(body_rot), np.array([[body_offset, 0, 0]]).T)
    body_2 = np.matmul(inv(body_rot), np.array([[body_offset / 2,
                                                 -body_offset * sin(pi / 3),
                                                 0]]).T)
    body_3 = np.matmul(inv(body_rot), np.array([[-body_offset / 2,
                                                 -body_offset * sin(pi / 3),
                                                 0]]).T)
    body_4 = np.matmul(inv(body_rot), np.array([[-body_offset, 0, 0]]).T)
    body_5 = np.matmul(inv(body_rot), np.array([[-body_offset / 2,
                                                 body_offset * sin(pi / 3),
                                                 0]]).T)

    body_model = np.concatenate((body_0.T, body_1.T, body_2.T, body_3.T,
                                 body_4.T, body_5.T, body_0.T), axis=0)
    translation = [[Tx, Ty, Tz],
                   [Tx, Ty, Tz],
                   [Tx, Ty, Tz],
                   [Tx, Ty, Tz],
                   [Tx, Ty, Tz],
                   [Tx, Ty, Tz],
                   [Tx, Ty, Tz]]

    body_model = body_model + translation

    return body_model
