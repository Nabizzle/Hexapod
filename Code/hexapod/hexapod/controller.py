from hexapod.leg import recalculateLegAngles, startLegPos, legModel
from hexapod.body import bodyPos
from hexapod.move import switchMode, emgToWalk, resetWalkStance, emgToTurn, resetTurnStance, walk
from hexapod.ssc32uDriver import anglesToSerial, connect, sendData
from time import sleep


def controller(mode):
    """controls the hexapod to walk or turn and send the commands"""
    port = connect('COM1') #connect to the servo controller
    # setup the starting robot positions
    body_model = bodyPos(pitch = 0, roll = 0, yaw = 0, Tx = 0, Ty = 0, Tz = 0, body_offset = 85)
    start_leg = startLegPos(body_model, start_radius = 150, start_height = 20)
    message = anglesToSerial(start_leg, 500, 2000) #get the serial message from the angles
    sendData(port, message) #send the serial message
    leg_model = legModel(start_leg, body_model)

    # iterate forever
    previous_step = 0
    previous_turn_angle = 0
    right_foot = True
    while True:
        if mode: #True = walk, False = turn
            [leg_model, right_foot, previous_step, positions] = emgToWalk(body_model, leg_model, right_foot, previous_step, max_distance = 30)
        else:
            [leg_model, right_foot, previous_turn_angle, positions] = emgToTurn(body_model, leg_model, right_foot, previous_turn_angle, max_turn_angle = 15)

        sendPositions(port, positions, body_model)

        if switchMode(0.75):
            if mode:
                [leg_model, right_foot, positions] = resetWalkStance(body_model, leg_model, right_foot, previous_step)
                previous_step = 0
            else:
                [leg_model, right_foot, positions] = resetTurnStance(body_model, leg_model, right_foot, previous_turn_angle)
            mode = not mode
            sendPositions(port, positions, body_model)


def sit(port):
    """tells the Hexapod to sit with its body on the ground"""
    body_model = bodyPos(pitch = 0, roll = 0, yaw = 0, Tx = 0, Ty = 0, Tz = 0, body_offset = 85)
    sit_leg = startLegPos(body_model, start_radius = 90, start_height = 10)
    message = anglesToSerial(sit_leg, 500, 2000) #get the serial message from the angles
    sendData(port, message) #send the serial message


def stand():
    """tells the hexapod to stand for the first time"""
    # controls the hexapod to walk or turn and send the commands
    port = connect('COM4') #connect to the servo controller
    # setup the starting robot positions
    body_model = bodyPos(pitch = 0, roll = 0, yaw = 0, Tx = 0, Ty = 0, Tz = 0, body_offset = 85)
    start_leg = startLegPos(body_model, start_radius = 180, start_height = 60)
    message = anglesToSerial(start_leg, 500, 2000) #get the serial message from the angles
    sendData(port, message) #send the serial message


def walkCycle(port, body_model, leg_model, distance, angle):
    """Tells the hexapod to walk a specified distance without the need for EMG"""
    positions = walk(leg_model, distance, angle)
    sendPositions(port, positions, body_model)


def sendPositions(port, positions, body_model):
    """
    Send each position in a set of hexapod command positions to the servo
    controller
    """
    for position in positions:
        angles = recalculateLegAngles(position, body_model) #convert the feet positions to angles
        message = anglesToSerial(angles, 1000, 2000) #get the serial message from the angles
        sendData(port, message) #send the serial message
        sleep(0.005)
    return True
