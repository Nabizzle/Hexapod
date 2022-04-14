from math import degrees, radians, sin, cos, atan2, sqrt
import numpy as np
from hexapod.leg import getFeetPos
from hexapod.piToPi import pollEMG

def stepForward(step_angle = 90, distance = 30, step_height = 15, right_foot = True):
    # Calculate the x, y, and z position updates to move in a step in a direction
    z_resolution = 1 # the forward distance of each sub step.

    z = np.array([-(i ** 2) / 4 + ((step_height) ** 2) / 4 for i in np.arange (- step_height, step_height + z_resolution, z_resolution)])
    x = np.linspace(0, distance * cos(radians(step_angle)), z.size)
    y = np.linspace(0, distance * sin(radians(step_angle)), z.size)
    lead_foot = np.dstack((x, y, z)).reshape(z.size, 1, 3)
    dragging_foot = np.dstack((- x, - y, np.zeros(z.size))).reshape(z.size, 1, 3)

    #define legs 0, 2, and 4 as the right legs and legs 1, 3, 5 as the left legs
    if right_foot: #right foot
        feet = np.concatenate((lead_foot, dragging_foot, lead_foot, dragging_foot, lead_foot, dragging_foot), axis = 1)
    else:
        feet = np.concatenate((dragging_foot, lead_foot, dragging_foot, lead_foot, dragging_foot, lead_foot), axis = 1)

    return feet

def stepTurnFoot(foot_x, foot_y, foot_z, step_angle = 15, step_height = 15, right_foot = True):
    z_resolution = 1 # the forward distance of each sub step.
    radius = sqrt(foot_x ** 2 + foot_y ** 2)
    foot_angle = degrees(atan2(foot_y, foot_x))

    z = np.array([-(i ** 2) / 4 + ((step_height) ** 2) / 4 + foot_z for i in np.arange (- step_height, step_height + z_resolution, z_resolution)])
    x = np.empty(z.size)
    y = np.empty(z.size)
    angles = np.linspace(foot_angle, foot_angle + step_angle, z.size)
    for i, angle in enumerate(angles):
        x[i] = radius * cos(radians(angle))
        y[i] = radius * sin(radians(angle))

    #define legs 0, 2, and 4 as the right legs and legs 1, 3, 5 as the left legs
    if right_foot: #right foot
        angles = np.linspace(foot_angle, foot_angle + step_angle, z.size)
    else:
        angles = np.linspace(foot_angle, foot_angle - step_angle, z.size)
        z = np.zeros(z.size) + foot_z

    for i, angle in enumerate(angles):
        x[i] = radius * cos(radians(angle))
        y[i] = radius * sin(radians(angle))

    return np.dstack((x, y, z)).reshape(z.size, 1, 3)

def stepTurn(feet_pos, step_angle = 15, step_height = 15, right_foot = True):
    for i in range(6):
        footstep = stepTurnFoot(foot_x = feet_pos[i, 0], foot_y = feet_pos[i, 1], foot_z = feet_pos[i, 2], step_angle = step_angle, step_height = step_height, right_foot = right_foot)
        right_foot = not right_foot
        if i == 0:
            previous_foot = footstep
        else:
            previous_foot = np.concatenate((previous_foot, footstep), axis = 1)

    return previous_foot

def walk(leg_model, distance = 30, angle = 90):
    # Creates a series of foot positions to use in telling the robot to walk in a directino
    max_step_size = 30 #Maximum step distance
    if distance <= 0: #raise an error if the robot is not commanded to move a positive distance
        raise ValueError("distance must be a positive distance")
    else: #find the number of steps to take
        steps = int(distance / max_step_size)
        if distance % max_step_size > 0:
            steps += 1

    right_foot = True; #If the right foot is moving forward
    remaining_distance = distance #Sets the remaining distance to move forward as the full distane to move
    for i in range(steps): #iterate over the number of steps to take
        if remaining_distance <= max_step_size: #if the remaining distance to move is less than the max step size, then move the robot remaining distance
            if steps == 1: #if this is the only step then the feet only needs to move forward the remaining distance
                temp_walk_positions = stepForward(step_angle = angle, distance = remaining_distance, right_foot = right_foot)
            else: #if this is not the first step the robot needs to move forward the max step size first to bring the robot to a neutral position before moving the rest of the distance.
                temp_walk_positions = stepForward(step_angle = angle, distance = remaining_distance + max_step_size, right_foot = right_foot)

            try: #try to get the current feet positions from the last stepped position
                feet_positions = walk_positions[-1, :, :]
            except: #if there was not a step yet get the feet positions from the leg model
                feet_positions = getFeetPos(leg_model)
            for j in range(temp_walk_positions.shape[0]): #add all of the feet positions to the walk
                temp_walk_positions[j, :, :] = temp_walk_positions[j, :, :] + feet_positions
            try: # try to add the next step to the walk
                walk_positions = np.concatenate((walk_positions, temp_walk_positions), axis = 0)
            except: #if this is the first step, create the walk array with the first step
                walk_positions = temp_walk_positions
            right_foot = not right_foot #switch which foot steps forward
            break

        elif i == 0: #if this is the first of more than one step, move forward the max distance
            walk_positions = stepForward(step_angle = angle, distance = max_step_size, right_foot = right_foot)
            feet_positions = getFeetPos(leg_model)
            for j in range(walk_positions.shape[0]):
                walk_positions[j, :, :] = walk_positions[j, :, :] + feet_positions
            remaining_distance -= max_step_size #reduce the remaining distance by the max step size
            right_foot = not right_foot

        else: #if this is not the first step and the next step is more than the max distance, move the legs forward by twice the max distance to reset and then move forward the max distance
            temp_walk_positions = stepForward(step_angle = angle, distance = max_step_size * 2, right_foot = right_foot)

            try:
                feet_positions = walk_positions[-1, :, :]
            except:
                feet_positions = getFeetPos(leg_model)
            for j in range(temp_walk_positions.shape[0]):
                temp_walk_positions[j, :, :] = temp_walk_positions[j, :, :] + feet_positions
            walk_positions = np.concatenate((walk_positions, temp_walk_positions), axis = 0)
            remaining_distance -= max_step_size
            right_foot = not right_foot

    temp_walk_positions = stepForward(step_angle = angle, distance = remaining_distance, right_foot = right_foot) #reset the position of the robot by moving the last step distance

    feet_positions = walk_positions[-1, :, :]
    for j in range(temp_walk_positions.shape[0]):
        temp_walk_positions[j, :, :] = temp_walk_positions[j, :, :] + feet_positions
    walk_positions = np.concatenate((walk_positions, temp_walk_positions), axis = 0)
    return walk_positions

def turn(leg_model, turn_angle = 60):
    # Creates the series of foot positions to turn the hexapod about the z axis.
    max_turn_angle = 15 #sets the maximum angle to turn by.
    if turn_angle == 0: #Raise an error is the robot is not commanded to move a non zero angle
        raise ValueError("turn angle must be a number larger than 0.")
    else: #find the number of steps to take
        steps = int(abs(turn_angle / max_turn_angle))
        if abs(turn_angle % max_turn_angle) > 0:
            steps += 1

    right_foot = True; #If the right foot is moving forward
    remaining_turn_distance = turn_angle #Sets the remaining turn distance to the full turn
    for i in range(steps): #iterate over the number of steps to take
        if abs(remaining_turn_distance) <= max_turn_angle: #if the remaining turn distance to move is less than the max turn angle, then turn the robot the remaining angle
            try: #try to get the current feet positions from the last stepped position
                feet_positions = turn_positions[-1, :, :]
            except: #if there was not a step yet get the feet positions from the leg model
                feet_positions = getFeetPos(leg_model)

            if steps == 1: #if this is the only step then the feet only need to turn the robot the remaining turn distance
                temp_turn_positions = stepTurn(feet_positions, step_angle = remaining_turn_distance, right_foot = right_foot)
            else: #if this is not the first step the robot needs to move forward the max turn size first to bring the robot to a neutral position before moving the rest of the distance.
                temp_turn_positions = stepTurn(feet_positions, step_angle = np.sign(remaining_turn_distance) * (abs(remaining_turn_distance) + max_turn_angle), right_foot = right_foot)

            try: # try to add the next step to the walk
                turn_positions = np.concatenate((turn_positions, temp_turn_positions), axis = 0)
            except: #if this is the first step, create the walk array with the first step
                turn_positions = temp_turn_positions
            right_foot = not right_foot #switch which foot steps forward
            break

        elif i == 0: #if this is the first of more than one step, turn the max angle
            feet_positions = getFeetPos(leg_model)
            turn_positions = stepTurn(feet_positions, step_angle = np.sign(remaining_turn_distance) * max_turn_angle, right_foot = right_foot)
            remaining_turn_distance -= np.sign(remaining_turn_distance) * max_turn_angle #reduce the remaining distance by the max step size
            right_foot = not right_foot

        else: #if this is not the first step and the next step is more than the max angle, move the legs forward by twice the max angle to reset and then turn the max distance
            try:
                feet_positions = turn_positions[-1, :, :]
            except:
                feet_positions = getFeetPos(leg_model)

            temp_turn_positions = stepTurn(feet_positions, step_angle = np.sign(remaining_turn_distance) * max_turn_angle * 2, right_foot = right_foot)

            turn_positions = np.concatenate((turn_positions, temp_turn_positions), axis = 0)
            remaining_turn_distance -= np.sign(remaining_turn_distance) * max_turn_angle
            right_foot = not right_foot

    feet_positions = turn_positions[-1, :, :]
    temp_turn_positions = stepTurn(feet_positions, step_angle = remaining_turn_distance, right_foot = right_foot) #reset the position of the robot by moving the last step distance

    turn_positions = np.concatenate((turn_positions, temp_turn_positions), axis = 0)
    return turn_positions


def emgToWalk(leg_model, right_foot, previous_step, max_distance = 30):
    # Walks a dynamic distance based a normalized EMG input.
    #call a function to poll for forearm emg values from the raspberry pi zero
    [fcr_emg, edc_emg] = pollEMG()
    emg = fcr_emg - edc_emg #finds the difference between EMG signals to move forward or backwards

    distance = round(max_distance * abs(emg)) # find a integer distance to move that is a percentage of the max distance.
    walk_positions = stepForward(step_angle = np.sign(emg) * 90, distance = distance + previous_step, right_foot = right_foot)
    feet_positions = getFeetPos(leg_model)
    for i in range(walk_positions.shape[0]): #add all of the feet positions to the walk
        walk_positions[i, :, :] = walk_positions[i, :, :] + feet_positions

    # TODO: Write a function to use the walk positions to make a movement

    previous_step = distance
    right_foot = not right_foot
    return[leg_model, right_foot, previous_step]

def emgToTurn(leg_model, right_foot, previous_turn_angle, max_turn_angle = 15):
    # Turns a dynamic angle based on a normalized EMG input
    #call a function to poll for forearm emg values from the raspberry pi zero
    [fcr_emg, edc_emg] = pollEMG()
    emg = fcr_emg - edc_emg #finds the difference between EMG signals to move right or left

    turn_angle = round(max_turn_angle * emg)
    feet_positions = getFeetPos(leg_model)

    turn_positions = stepTurn(feet_positions, step_angle = np.sign(turn_angle) * (abs(turn_angle) + previous_turn_angle), right_foot = right_foot)

    # TODO: Write a function to use the turn positions to make a movement

    previous_turn_angle = turn_angle
    right_foot = not right_foot

    return[leg_model, right_foot, previous_turn_angle]
    
def switchMode(fcr_emg, edc_emg, threshold):
    #if the user is cocontracting, tell the hexapod to switch walking modes.
    if fcr_emg > threshold and edc_emg > threshold:
        return True
    else:
        return False