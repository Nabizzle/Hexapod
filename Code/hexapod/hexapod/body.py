"""
Functions related to calculating the points in x, y, z space that make up the
body model.

Functions in this model are used to take in desired angles and translations to
orient the body in whatever way the user desires independent of the legs and
their movement.

Functions:
bodyPos: Creates a model of the body with input rotations and translations.
"""

import numpy as np
from numpy.linalg import inv
from numpy.typing import NDArray
from hexapod.rotation import xRot, yRot, zRot
from math import sin, pi


def bodyPos(pitch: float = 0, roll: float = 0, yaw: float = 0, Tx: float = 0,
            Ty: float = 0, Tz: float = 0, body_offset: float = 85) -> NDArray:
    """
    Creates a model of the body with input rotations and translations.

    Applies rotations in `pitch`, `roll`, and `yaw` and translations in x, y, 
    and z to the body. After this, it creates a model of the hexapod's body
    that represents the locations of the coax servo output shafts.

    Parameters
    ----------
    pitch: float, default=0
        The rotation of the body about the x axis.roll
    roll: float, default=0
        The rotation of the body about the y axis.
    yaw: float, default=0
        The rotation of the body about the z axis.
    Tx: float, default=0
        Translation of the body along the x axis.
    Ty: float, default=0
        Translation of the body along the y axis.
    Tz: float, default=0
        Translation of the body along the z axis.
    body_offset: float, default=85
        The distance in millimeters along the x axis of the second coax angle
        servo from the center of the hexapod when the body has no translations
        or rotations.

    Returns
    -------
    body_model: NDArray
        A 7x3 numpy array that represents the locations of the coax servos as
        x, y, and z points.

    Notes
    -----
    The seventh point in the model is the same as the first and was used when plotting the model, but is not needed in controlling the hexapod.
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
