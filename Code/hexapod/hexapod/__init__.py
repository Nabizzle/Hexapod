"""
A library of funtions to control an 18 DoF hexapod

This library contains code to create a computational model of the body and
legs of the hexapod so that those models can control a real hexapod's 18
servos. The library also has a driver to convert the model angles to 18 servo
commands. The hexapod can run automatically walking and turning based on code
or can take in two EMG streams to direct the amount to walk and turn.

Modules
-------
body: Functions that relate to creating the model of the hexapod body.
controller:
    Scripts for directly controlling the hexapod aggregating the total work of
    the full library.
leg:
    These are functions for finding and refinding the positions of the legs
    from their servo angles or to do the reverse in finding the servo angles
    from the end positions of the hexapod feet.
move:
    These are a collection of functions to find how the hexapod will take
    steps to move linearlly in any direction and to turn itself in the x-y
    plane.
piTopi: Scripts used in the communications with and between Raspberry Pis.
rotation:
    These are the functions to find the rotation of any point around an axis
    by an angle in degrees.
ssc32uDriver:
    Functions to communicate with the Lynxmotion SSC-32U to drive the 18
    servos of the hexapod.

Functions
---------
body.bodyPos:
    Applies rotations in pitch, roll, and yaw and translations in x, y, and
    z to the body.
controller.controller: Controls the hexapod to walk or turn based on EMG.
controller.sit: Tells the Hexapod to sit with its body on the ground.
controller.stand: Tells the hexapod to stand in the neutral position.
controller.walkCycle:
    Tells the hexapod to walk a specified distance without the need for EMG
controller.sendPositions:
    Send each position in a set of hexapod command positions to the servo
    controller
leg.legPos: Finds the positions for the leg segments.
leg.legAngle: Finds the angles for the coax, femur, and tibia leg segments.
leg.recalculateLegAngles: Finds the coax, femur, and tibia angles of each leg.
leg.startLegPos: Find the neutral position of the hexapod.
leg.legModel:
    Generates the model of the legs based on the servo angles of the legs.
leg.getFeetPos: Ouput the x, y, z position of the feet of the hexapod.
move.stepForward:
    Calculate the x, y, and z positions to move in a step in a direction.
move.stepTurnFoot:
    Calculate the offset of a foor when turning the hexapod about an angle.
move.stepTurn:
    Calcluate the absolute positions of each foot of the hexapod when
    turning about an angle
move.walk
    Creates a series of foot positions to use in telling the robot to walk
    in a direction
move.turn
    Creates the series of foot positions to turn the hexapod about the z axis.
move.emgToWalk: Walks a dynamic distance based a normalized EMG input.
move.resetWalkStance:
    Takes the final step of the walk cycle by repeating the previous step
    with the opposite legs as the last step.
move.emgToTurn:Turns a dynamic angle based on a normalized EMG input.
move.resetTurnStance:
    Takes the final step of the turn cycle by repeating the previous step
    with the opposite legs as the last step.
move.switchMode:
    Switches walking modesIf the user is cocontracting their muscles.
move.pollEMG: Get EMG signals and normalize them.
piTopi.recieveEMG: Pull EMG from the Raspberry Pi Zero W
rotation.xRot: Return the rotation matrix for a rotation about the x axis.
rotation.yRot: Return the rotation matrix for a rotation about the y axis.
rotation.zRot: Return the rotation matrix for a rotation about the z axis.
ssc32uDriver.angleToPW:
    Convert the input angle in degrees to the pulse width in microseconds to
    command that angle.
ssc32uDriver.anglesToSerial:
    Converts an array of servo angles to the formatted serial command for
    the Lynxmotiohn SSC-32U.
ssc32uDriver.connect:
    Tries to open a serial port with the Lynxmotiohn SSC-32U on the desired
    COM port.
ssc32uDriver.disconnect: Disconnects from the serial port.
ssc32uDriver.sendData:
    Sends the byte string command for the servos to the Lynxmotiohn SSC-32U.
"""
