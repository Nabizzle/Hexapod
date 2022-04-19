from math import radians, cos, sin
import numpy as np
from numpy.typing import NDArray


def xRot(theta: float) -> NDArray:
    """Return the rotation matrix for a rotation about the x axis"""
    angle = radians(theta)
    mat = np.array([[1, 0, 0],
                   [0, cos(angle), sin(angle)],
                   [0, -sin(angle), cos(angle)]])
    return mat


def yRot(theta: float) -> NDArray:
    """Return the rotation matrix for a rotation about the y axis"""
    angle = radians(theta)
    mat = np.array([[cos(angle), 0, sin(angle)],
                   [0, 1, 0],
                   [-sin(angle), 0, cos(angle)]])
    return mat


def zRot(theta: float) -> NDArray:
    """Return the rotation matrix for a rotation about the z axis"""
    angle = radians(theta)
    mat = np.array([[cos(angle), sin(angle), 0],
                   [-sin(angle), cos(angle), 0],
                   [0, 0, 1]])
    return mat
