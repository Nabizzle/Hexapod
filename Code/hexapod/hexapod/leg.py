import numpy as np
from numpy.linalg import inv
from math import degrees, sin, cos, acos, atan2, sqrt, pi
from hexapod.rotation import yRot, zRot


def legPos(coax_angle, femur_angle, tibia_angle, body_model, leg_num, coax = 26.34, femur = 76.2, tibia = 88.32):
    """
    finds the positions for the coax, femur, and tibia leg segments and
    adds them to the body model
    """
    coax_rot = zRot(coax_angle)
    femur_rot = np.matmul(yRot(femur_angle), coax_rot)
    tibia_rot = np.matmul(yRot(tibia_angle), femur_rot)

    leg_coax = np.matmul(inv(coax_rot), np.array([[coax, 0, 0]]).T) + np.array([body_model[leg_num, :]]).T
    leg_femur = np.matmul(inv(femur_rot), np.array([[femur, 0, 0]]).T) + leg_coax
    leg_tibia = np.matmul(inv(tibia_rot), np.array([[0, 0, -tibia]]).T) + leg_femur

    leg_positions = np.concatenate((np.array([body_model[leg_num, :]]), leg_coax.T, leg_femur.T, leg_tibia.T), axis = 0)
    return leg_positions


def legAngle(x, y, z, coax = 26.34, femur = 76.2, tibia = 88.32):
    """finds the angles for the coax, femur, and tibia leg segments"""
    coax_angle = degrees(atan2(y, x))
    coax_rot = zRot(-coax_angle)
    leg_rotated = np.matmul(inv(coax_rot), np.array([[x, y, z]]).T)
    femur_angle = degrees(acos((tibia ** 2 - femur ** 2 - leg_rotated[2] ** 2 - (leg_rotated[0] - coax) ** 2) / (-2 * femur * (sqrt(leg_rotated[2] ** 2 + (leg_rotated[0] - coax) ** 2))))) - degrees(atan2(-leg_rotated[2], (leg_rotated[0] - coax)))
    tibia_angle = degrees(acos((leg_rotated[2] ** 2 + (leg_rotated[0] - coax) ** 2 - femur ** 2 - tibia ** 2) / (-2 * femur * tibia))) - 90

    if abs(coax_angle) <= 1e-10:
        coax_angle = 0

    if abs(femur_angle) <= 1e-10:
        femur_angle = 0

    if abs(tibia_angle) <= 1e-10:
        tibia_angle = 0

    return [coax_angle, femur_angle, tibia_angle]


def recalculateLegAngles(feet_positions, body_model):
    """
    Finds the coax, femur, and tibia angles of each leg based on the body
    model and feet positions of the hexapod
    """
    leg_angles = np.empty([6, 3])
    for i in range(6):
        leg_angles[i, :] = legAngle(feet_positions[i, 0] - body_model[i, 0], feet_positions[i, 1] - body_model[i, 1], feet_positions[i, 2] - body_model[i, 2])
    return leg_angles


def startLegPos(body_model, start_radius = 150, start_height = 20):
    """Create the starting angles of the legs on the hexapod based"""
    start_leg_pos = np.array([[start_radius * cos(pi / 3), start_radius * sin(pi / 3), - start_height],
                              [start_radius, 0, - start_height],
                              [start_radius * cos(- pi / 3), start_radius * sin(- pi / 3), - start_height],
                              [start_radius * cos(- 2 * pi / 3) , start_radius * sin(- 2 * pi / 3), - start_height],
                              [- start_radius, 0, - start_height],
                              [start_radius * cos(2 * pi / 3), start_radius * sin(2 * pi / 3), - start_height]])
    start_leg = recalculateLegAngles(start_leg_pos, body_model)
    return start_leg


def legModel(leg_angles, body_model):
    """Generates the model of the legs based on the servo angles of the legs."""
    leg_model = np.empty([4, 3, 6])
    for i in range(6):
        leg_model[:, :, i] = legPos(leg_angles[i][0], leg_angles[i][1], leg_angles[i][2], body_model, i) #coax angle, femur angle, tibia angle, model of the hexapod body, leg number
    return leg_model


def getFeetPos(leg_model):
    """
    return the current positions of the ends of the legs or where the feet
    of the hexapod currently are.
    """
    feet_positions = np.empty([6, 3])
    for i in range(6):
        feet_positions[i, :] = leg_model[3, :, i]
    return feet_positions
