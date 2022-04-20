"""
Scripts for controlling the hexapod.

These are scripts that control how the hexapod walks and turns. This module
combines the work of the rest of the library and as a result, imports most of
it to utilize most of the functions.

Functions
---------
controller: Controls the hexapod to walk or turn based on EMG.
sit: Tells the Hexapod to sit with its body on the ground.
stand: Tells the hexapod to stand in the neutral position.
walkCycle:
    Tells the hexapod to walk a specified distance without the need for EMG
sendPositions: Send each position in a set to the servo controller.
"""
from hexapod.leg import recalculateLegAngles, startLegPos, legModel
from hexapod.body import bodyPos
from hexapod.move import (switchMode, emgToWalk, resetWalkStance, emgToTurn,
                          resetTurnStance, walk)
from hexapod.ssc32uDriver import anglesToSerial, connect, sendData
from numpy.typing import NDArray
from typing import Any
from time import sleep


def controller(mode: bool) -> None:
    """
    Controls the hexapod to walk or turn based on EMG.

    Takes in EMG signals and tells the hexapod how to walk or turn based on
    those EMG signals. The signals are sent over a COM port to the Lynxmotion
    SSC32U servo controller. This controller combines the work of the rest of
    the library to get the hexapod to move. This is the main function to run
    when using the hexapod for its intended purpose.

    Parameters
    ----------
    mode: bool
        This parameter determines if the hexapod is walking or turning. `mode`
        equalling 1 is walking and 0 is turning.

    Notes
    -----
    This function has the serial port to communicate to and the threshold
    of cocontraction to switch modes hardcoded.

    See Also
    --------
    move.switchMode:
        Sends a boolean based on EMG to indicate when to change movement
        modes.
    """
    port = connect('COM1')  # connect to the servo controller
    # setup the starting robot positions
    body_model = bodyPos(pitch=0, roll=0, yaw=0, Tx=0, Ty=0, Tz=0,
                         body_offset=85)
    start_leg = startLegPos(body_model, start_radius=150, start_height=20)
    # get the serial message from the angles
    message = anglesToSerial(start_leg, 500, 2000)
    sendData(port, message)  # send the serial message
    leg_model = legModel(start_leg, body_model)

    # iterate forever
    previous_step = 0
    previous_turn_angle = 0
    right_foot = True
    while True:
        if mode:  # True = walk, False = turn
            [leg_model, right_foot, previous_step, positions] =\
                emgToWalk(body_model, leg_model, right_foot, previous_step,
                          max_distance=30)
        else:
            [leg_model, right_foot, previous_turn_angle, positions] =\
                emgToTurn(body_model, leg_model, right_foot,
                          previous_turn_angle, max_turn_angle=15)

        sendPositions(port, positions, body_model)

        if switchMode(0.75):
            if mode:
                [leg_model, right_foot, positions] =\
                    resetWalkStance(body_model, leg_model, right_foot,
                                    previous_step)
                previous_step = 0
            else:
                [leg_model, right_foot, positions] =\
                    resetTurnStance(body_model, leg_model, right_foot,
                                    previous_turn_angle)
            mode = not mode
            sendPositions(port, positions, body_model)


def sit(port: Any) -> None:
    """
    Tells the Hexapod to sit with its body on the ground.

    Recreates the neutral hexapod positions, but with a lowered height so that
    the body of the hexapod touches the ground. This is the position that the
    hexapod should be turned off in.

    Parameters
    ----------
    port: Serial Port
        The COM port that the servo signals are sent over.
    """
    body_model = bodyPos(pitch=0, roll=0, yaw=0, Tx=0, Ty=0, Tz=0,
                         body_offset=85)
    sit_leg = startLegPos(body_model, start_radius=90, start_height=10)
    # get the serial message from the angles
    message = anglesToSerial(sit_leg, 500, 2000)
    sendData(port, message)  # send the serial message


def stand() -> None:
    """
    Tells the hexapod to stand in the neutral position.

    Recreates the neutral hexapod positions that occur at the beginning
    of the controller function.

    Notes
    -----
    This function assumes that there is already a connection to the serial
    port for the Lynxmotion SSC32U.

    See Also
    --------
    controller:
        Controls the hexapod to walk or turn based on EMG.
    """
    # controls the hexapod to walk or turn and send the commands
    port = connect('COM4')  # connect to the servo controller
    # setup the starting robot positions
    body_model = bodyPos(pitch=0, roll=0, yaw=0, Tx=0, Ty=0, Tz=0,
                         body_offset=85)
    start_leg = startLegPos(body_model, start_radius=180, start_height=60)
    # get the serial message from the angles
    message = anglesToSerial(start_leg, 500, 2000)
    sendData(port, message)  # send the serial message


def walkCycle(port: Any, body_model: NDArray, leg_model: NDArray,
              distance: float, angle: float) -> None:
    """
    Tells the hexapod to walk a specified distance without the need for EMG.

    This function just uses the move.walk function and sends its positions to
    the Lynxmotion SSC32U.

    Parameters
    ----------
    port: Serial Port
        The COM port that the servo signals are sent over.
    body_model: NDArray
        The 7x3 numpy array containing the locations of the coax servos.
    leg_model: NDArray
        the 4x3x6 numpy array that holds the locations of the coax, femur,
        and tibia servos as well as the feet end positions.
    distance: float
        The length in millimeters that the hexapod will walk. This distance is
        broken up into steps based on the max step size in move.walk
    angle: float
        The direction the hexapod will walk in. 90 degrees is forward.

    See Also
    --------
    move.walk
    """
    positions = walk(leg_model, distance, angle)
    sendPositions(port, positions, body_model)


def sendPositions(port: Any, positions: NDArray, body_model: NDArray) -> bool:
    """
    Send each position in a set to the servo controller.

    Takes a list of command positions and iterates through them to send each
    position as a servo command to the Lynxmotion SSC32U.

    Parameters
    ----------
    port: Serial Port
        The COM port that the servo signals are sent over
    positions: NDArray
        A numpy array of a set of foot positions to command the hexapod to.
        These foot positions are a 6x3 numpy array of x, y, z positions for
        the six legs and the number of 6x3 arrays in the positions matrix is
        determined by how far the hexapod walked or turned.
    body_model: NDArray
        The 7x3 numpy array containing the locations of the coax servos.

    Returns
    -------
    bool:
        The function returns true when finished.

    Notes
    -----
    In this function, the ssc32uDriver.anglesToSerial function does not have
    its speed or time inputs used as each change in position is so small, that
    using these parameters is unneeded. Also note that there is a 5ms delay
    between commands.
    """
    for position in positions:
        # convert the feet positions to angles
        angles = recalculateLegAngles(position, body_model)
        # get the serial message from the angles
        message = anglesToSerial(angles)
        sendData(port, message)  # send the serial message
        sleep(0.005)
    return True
