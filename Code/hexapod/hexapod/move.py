from math import degrees, radians, sin, cos, atan2, hypot
import numpy as np
from hexapod.leg import getFeetPos, recalculateLegAngles, legModel
from hexapod.piToPi import pollEMG


def stepForward(step_angle=90, distance=30, step_height=15, right_foot=True):
    """Calculate the x, y, and z position updates to move in a step in a direction"""
    z_resolution = 1  # the forward distance of each sub step.

    z = np.array([-(i ** 2) / 4 + ((step_height) ** 2) / 4
                  for i in np.arange(-step_height, step_height + z_resolution,
                  z_resolution)])
    x = np.linspace(0, distance * cos(radians(step_angle)), z.size)
    y = np.linspace(0, distance * sin(radians(step_angle)), z.size)
    lead_foot = np.dstack((x, y, z)).reshape(z.size, 1, 3)
    dragging_foot =\
        np.dstack((- x, - y, np.zeros(z.size))).reshape(z.size, 1, 3)

    # legs 0, 2, and 4 are the right legs and legs 1, 3, 5 as the left legs
    if right_foot:  # right foot
        feet = np.concatenate((lead_foot, dragging_foot, lead_foot,
                               dragging_foot, lead_foot, dragging_foot),
                              axis = 1)
    else:
        feet = np.concatenate((dragging_foot, lead_foot, dragging_foot,
                               lead_foot, dragging_foot, lead_foot), axis = 1)

    return feet


def stepTurnFoot(foot_x, foot_y, foot_z, step_angle=15, step_height=15,
                 right_foot = True):
    """Calculate the offset of a foor when turning the hexapod about an angle"""
    z_resolution = 1  # the forward distance of each sub step.
    radius = hypot(foot_x, foot_y)
    foot_angle = degrees(atan2(foot_y, foot_x))

    z = np.array([-(i ** 2) / 4 + ((step_height) ** 2) / 4 + foot_z
                  for i in np.arange(-step_height, step_height + z_resolution,
                  z_resolution)])
    x = np.empty(z.size)
    y = np.empty(z.size)
    angles = np.linspace(foot_angle, foot_angle + step_angle, z.size)
    for i, angle in enumerate(angles):
        x[i] = radius * cos(radians(angle))
        y[i] = radius * sin(radians(angle))

    # legs 0, 2, and 4 are the right legs and legs 1, 3, 5 as the left legs
    if right_foot:  # right foot
        angles = np.linspace(foot_angle, foot_angle + step_angle, z.size)
    else:
        angles = np.linspace(foot_angle, foot_angle - step_angle, z.size)
        z = np.zeros(z.size) + foot_z

    for i, angle in enumerate(angles):
        x[i] = radius * cos(radians(angle))
        y[i] = radius * sin(radians(angle))

    return np.dstack((x, y, z)).reshape(z.size, 1, 3)


def stepTurn(feet_pos, step_angle=15, step_height=15, right_foot=True):
    """
    Calcluate the absolute positions of each foot of the hexapod when
    turning about an angle
    """
    for i in range(6):
        footstep = stepTurnFoot(foot_x=feet_pos[i, 0],
                                foot_y=feet_pos[i, 1],
                                foot_z=feet_pos[i, 2],
                                step_angle=step_angle,
                                step_height=step_height,
                                right_foot=right_foot)
        right_foot = not right_foot
        if i == 0:
            previous_foot = footstep
        else:
            previous_foot = np.concatenate((previous_foot, footstep), axis = 1)

    return previous_foot


def walk(leg_model, distance=30, angle=90):
    """
    Creates a series of foot positions to use in telling the robot to walk
    in a direction
    """
    max_step_size = 30  # Maximum step distance
    # raise an error if the robot is not commanded to move a positive distance
    if distance <= 0:
        raise ValueError("distance must be a positive distance")
    steps = int(distance / max_step_size)
    if distance % max_step_size > 0:
        steps += 1

    right_foot = True  # If the right foot is moving forward
    # Sets the remaining distance to move forward as the full distane to move
    remaining_distance = distance
    for i in range(steps):  # iterate over the number of steps to take
        # if the remaining distance to move is less than the max step size,
        # then move the robot remaining distance
        if remaining_distance <= max_step_size:
            # if this is the only step then the feet only needs to move
            # forward the remaining distance
            if steps == 1:
                temp_walk_positions = stepForward(step_angle=angle,
                                                  distance=remaining_distance,
                                                  right_foot=right_foot)
            # if this is not the first step the robot needs to move forward
            # the max step size first to bring the robot to a neutral position
            # before moving the rest of the distance.
            else:
                temp_walk_positions = stepForward(step_angle=angle,
                                                  distance=remaining_distance
                                                  + max_step_size,
                                                  right_foot=right_foot)

            # Get the current feet positions from the last stepped position
            if 'walk_positions' in locals():
                feet_positions = walk_positions[-1, :, :]
            # if there was not a step yet get the feet positions from the leg
            # model
            else:
                feet_positions = getFeetPos(leg_model)
            # add all of the feet positions to the walk
            for j in range(temp_walk_positions.shape[0]):
                temp_walk_positions[j, :, :] = temp_walk_positions[j, :, :]\
                    + feet_positions
            # try to add the next step to the walk
            if 'walk_positions' in locals():
                walk_positions = np.concatenate((walk_positions,
                                                 temp_walk_positions),
                                                axis = 0)
            # create walk array with the first step
            else:
                walk_positions = temp_walk_positions
            right_foot = not right_foot  # switch which foot steps forward
            break

        # if this is the first of more than one step, move forward the max
        # distance
        if i == 0:
            walk_positions = stepForward(step_angle=angle,
                                         distance=max_step_size,
                                         right_foot=right_foot)
            feet_positions = getFeetPos(leg_model)
            for j in range(walk_positions.shape[0]):
                walk_positions[j, :, :] = walk_positions[j, :, :]\
                    + feet_positions
            # reduce the remaining distance by the max step size
            remaining_distance -= max_step_size
            right_foot = not right_foot

        # if this is not the first step and the next step is more than the max
        # distance, move the legs forward by twice the max distance to reset
        # and then move forward the max distance
        else:
            temp_walk_positions = stepForward(step_angle=angle,
                                              distance=max_step_size * 2,
                                              right_foot=right_foot)

            if 'walk_positions' in locals():
                feet_positions = walk_positions[-1, :, :]
            else:
                feet_positions = getFeetPos(leg_model)
            for j in range(temp_walk_positions.shape[0]):
                temp_walk_positions[j, :, :] = temp_walk_positions[j, :, :]\
                    + feet_positions
            walk_positions = np.concatenate((walk_positions,
                                             temp_walk_positions),
                                            axis = 0)
            remaining_distance -= max_step_size
            right_foot = not right_foot

    # reset the position of the robot by moving the last step distance
    temp_walk_positions = stepForward(step_angle=angle,
                                      distance=remaining_distance,
                                      right_foot=right_foot)

    feet_positions = walk_positions[-1, :, :]
    for j in range(temp_walk_positions.shape[0]):
        temp_walk_positions[j, :, :] = temp_walk_positions[j, :, :]\
            + feet_positions
    walk_positions = np.concatenate((walk_positions,
                                     temp_walk_positions),
                                    axis = 0)
    return walk_positions


def turn(leg_model, turn_angle = 60):
    """Creates the series of foot positions to turn the hexapod about the z axis."""
    max_turn_angle = 15  # sets the maximum angle to turn by.
    # Raise an error is the robot is not commanded to move a non zero angle
    if turn_angle == 0:
        raise ValueError("turn angle must be a number larger than 0.")
    steps = int(abs(turn_angle / max_turn_angle))
    if abs(turn_angle % max_turn_angle) > 0:
        steps += 1

    right_foot = True  # If the right foot is moving forward
    # Sets the remaining turn distance to the full turn
    remaining_turn_distance = turn_angle
    for i in range(steps):  # iterate over the number of steps to take
        # if the remaining turn distance to move is less than the max turn
        # angle, then turn the robot the remaining angle
        if abs(remaining_turn_distance) <= max_turn_angle:
            # Get the current feet positions from the last stepped position
            if 'turn_positions' in locals():
                feet_positions = turn_positions[-1, :, :]
            # if there was not a step yet get feet positions from the leg model
            else:
                feet_positions = getFeetPos(leg_model)

            # if this is the only step then the feet only need to turn the
            # robot the remaining turn distance
            if steps == 1:
                temp_turn_positions =\
                    stepTurn(feet_positions,
                             step_angle=remaining_turn_distance,
                             right_foot=right_foot)
            # if this is not the first step the robot needs to move forward
            # the max turn size first to bring the robot to a neutral position
            # before moving the rest of the distance.
            else:
                temp_turn_positions =\
                    stepTurn(feet_positions,
                             step_angle=np.sign(remaining_turn_distance)
                             * (abs(remaining_turn_distance)
                                + max_turn_angle),
                             right_foot=right_foot)

            # try to add the next step to the walk
            if 'turn_positions' in locals():
                turn_positions =\
                    np.concatenate((turn_positions,
                                    temp_turn_positions),
                                   axis = 0)
            # if this is the first step, create walk array with the first step
            else:
                turn_positions = temp_turn_positions
            right_foot = not right_foot  # switch which foot steps forward
            break

        # if this is the first of more than one step, turn the max angle
        if i == 0:
            feet_positions = getFeetPos(leg_model)
            turn_positions =\
                stepTurn(feet_positions,
                         step_angle=np.sign(remaining_turn_distance)
                         * max_turn_angle,
                         right_foot=right_foot)
            # reduce the remaining distance by the max step size
            remaining_turn_distance -=\
                np.sign(remaining_turn_distance) * max_turn_angle
            right_foot = not right_foot
        # if this is not the first step and the next step is more than the max
        # angle, move the legs forward by twice the max angle to reset and
        # then turn the max distance
        else:
            if 'turn_positions' in locals():
                feet_positions = turn_positions[-1, :, :]
            else:
                feet_positions = getFeetPos(leg_model)

            temp_turn_positions =\
                stepTurn(feet_positions,
                         step_angle=np.sign(remaining_turn_distance)
                         * max_turn_angle * 2,
                         right_foot=right_foot)

            turn_positions = np.concatenate((turn_positions,
                                             temp_turn_positions),
                                            axis = 0)
            remaining_turn_distance -=\
                np.sign(remaining_turn_distance) * max_turn_angle
            right_foot = not right_foot

    feet_positions = turn_positions[-1, :, :]
    # reset the position of the robot by moving the last step distance
    temp_turn_positions =\
        stepTurn(feet_positions,
                 step_angle=remaining_turn_distance,
                 right_foot=right_foot)

    turn_positions = np.concatenate((turn_positions,
                                     temp_turn_positions),
                                    axis = 0)
    return turn_positions


def emgToWalk(body_model, leg_model, right_foot, previous_step,
              max_distance=30):
    """Walks a dynamic distance based a normalized EMG input."""
    # call a function to poll for forearm emg values from the raspberry pi zero
    [fcr_emg, edc_emg] = pollEMG()
    # finds the difference between EMG signals to move forward or backwards
    emg = fcr_emg - edc_emg

    # find a integer distance to move that is a percentage of the max distance.
    distance = round(max_distance * emg)
    walk_positions = stepForward(step_angle=90,
                                 distance=distance + previous_step,
                                 right_foot=right_foot)
    feet_positions = getFeetPos(leg_model)
    # add all of the feet positions to the walk
    for i in range(walk_positions.shape[0]):
        walk_positions[i, :, :] = walk_positions[i, :, :] + feet_positions

    previous_step = distance
    right_foot = not right_foot
    leg_model = legModel(recalculateLegAngles(walk_positions[-1, :, :],
                                              body_model), body_model)
    return[leg_model, right_foot, previous_step, walk_positions]


def resetWalkStance(body_model, leg_model, right_foot, previous_step):
    """
    Takes the final step of the walk cycle by repeating the previous step
    with the opposite legs as the last step.
    """
    walk_positions = stepForward(step_angle = 90, distance = previous_step,
                                 right_foot = right_foot)
    feet_positions = getFeetPos(leg_model)
    # add all of the feet positions to the walk
    for i in range(walk_positions.shape[0]):
        walk_positions[i, :, :] = walk_positions[i, :, :] + feet_positions

    leg_model = legModel(recalculateLegAngles(walk_positions[-1, :, :],
                                              body_model), body_model)
    right_foot = not right_foot
    return[leg_model, right_foot, walk_positions]


def emgToTurn(body_model, leg_model, right_foot, previous_turn_angle,
              max_turn_angle=15):
    """Turns a dynamic angle based on a normalized EMG input"""
    # call a function to poll for forearm emg values from the raspberry pi zero
    [fcr_emg, edc_emg] = pollEMG()
    # finds the difference between EMG signals to move right or left
    emg = fcr_emg - edc_emg

    turn_angle = round(max_turn_angle * emg)
    feet_positions = getFeetPos(leg_model)

    turn_positions =\
        stepTurn(feet_positions,
                 step_angle=np.sign(turn_angle) * (abs(turn_angle)
                                                   + previous_turn_angle),
                 right_foot = right_foot)

    previous_turn_angle = turn_angle
    right_foot = not right_foot
    leg_model = legModel(recalculateLegAngles(turn_positions[-1, :, :],
                                              body_model), body_model)

    return [leg_model, right_foot, previous_turn_angle, turn_positions]


def resetTurnStance(body_model, leg_model, right_foot, previous_turn_angle):
    """
    Takes the final step of the turn cycle by repeating the previous step
    with the opposite legs as the last step.
    """
    feet_positions = getFeetPos(leg_model)

    turn_positions = stepTurn(feet_positions,
                              step_angle=previous_turn_angle,
                              right_foot=right_foot)

    right_foot = not right_foot
    leg_model = legModel(recalculateLegAngles(turn_positions[-1, :, :],
                                              body_model), body_model)

    return [leg_model, right_foot, turn_positions]


def switchMode(threshold):
    """if the user is cocontracting, tell the hexapod to switch walking modes."""
    [fcr_emg, edc_emg] = pollEMG()

    return bool(fcr_emg > threshold and edc_emg > threshold)
