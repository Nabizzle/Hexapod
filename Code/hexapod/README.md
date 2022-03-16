# Description of Each Script

Each description below is of each function in the computational model which will be transferred over to the physical hexapod.

## Rotation Matrices

These are the functions to find the rotation of any point around an axis by an angle in degrees.

### xRot

Rotation about the x axis by an angle in degrees.

### yRot

Rotation about the y axis by an angle in degrees.

### zRot

Rotation about the z axis by an angle in degrees.

## Body Functions

These are functions that relate to moving and creating the model of the hexapod body

### bodyPos

Creates a hexagon model of the body with the given pitch, roll, and yaw angles and the given x, y, and z translations from the center point of the coordinate axes.

## Leg Functions

These are functions for finding and refinding the positions of the legs from their servo angles or to do the reverse in finding the servo angles from the end positions of the hexapod feet.

### legPos

This function finds the positions of each segment of the leg from its servo angles and leg number and then outputs those locations to be added to the larger model of the legs.

### legAngle

Finds the servo angles of the leg based on the x, y, and z positions of the end contact point of the leg. This function then returns the servo angles as an output.

### startLegPos

Takes the body model and the starting outer radius the legs make on the robot as wel as the starting height of the robot off of the ground to find the starting servo angles of all of the legs. This function then outputs those angles as an array.

### getFeetPos

This function outputs the contact points of each leg with the ground as x, y, and z coordinates in an array.

### recalculateLegAngles

Takes an input of the current body model and current contact points of the legs to calculate the servo angles of all of the legs. This function outputs the servo angles of all of the legs.

### legModel

Takes in the servo angles of all of the legs as an array and calculates the positions of each leg segement to combine into a model of the legs.

## Movement Functions

These are a collection of functions to find how the hexapod will take steps to move linearlly in any direction and to turn itself in the x-y plane.

### stepForward

This function takes in an angle in the x-y plane to move in, a distance to move (in the same units as the model as a whole), a height to move each leg vertically, and a boolean to determine if the "right" legs are moving or the other legs are moving. The "right" legs are legs 0, 2, and 4 or every other leg starting from the front right leg of the robot and moving clockwise around the robot looking from above. This function then outputs the relative distance each foot needs to move compared to where it started to make a step in the desired direction. In order to move, one set of legs will lift up and the other will drag on the ground.

### stepTurnFoot

This function takes the x, y, and z position of a contact point of a leg along with the angle to turn to, the height to move the legs up to, and a boolean to determine if the "right" legs are moving or the other legs are moving. The "right" legs are legs 0, 2, and 4 or every other leg starting from the front right leg of the robot and moving clockwise around the robot looking from above. This funciton then returns the abolute location of each foot. **NOTE** that this is opposed to the relative position in the previous function as each foot can not move by the same relative amount to turn the robot. In order to move, one set of legs will lift up and the other will drag on the ground.

### stepTurn

This funciton takes in the contact positions of all of the legs, the angle to step to, the vertical height of the step, and a boolean to determine if the "right" legs are moving or the other legs are moving. The "right" legs are legs 0, 2, and 4 or every other leg starting from the front right leg of the robot and moving clockwise around the robot looking from above. This function then feeds each foot position into the previous funciton to find the absolute position of each foot.

## Movement Cycles

These are functions to create a walk and turn cycle.

### walk

Takes the body and leg models, a figure to plot on, a linear distance to move, and an angle to move in in the x-y frome. This function takes the distance to move and breaks it into steps to move based on a maximum step size. The cycle ends with the hexapod moving to the neutral position. This function requires a positive distance to move. Below is an example of this walk cycle.

<img src="https://github.com/Nabizzle/Hexapod/blob/main/Docs/Media/Hexapod_Walk.gif" width="500">

### turn

Takes the body and leg models, a figure to plot on, and an angle to turn to. This function breaks the turn into steps based on the maximum turn angle. The cycle ends with the hexapod moving to the neutral position. This function requires a non-zero angle to turn to (positive angles are left turns). Below is an example of this turn cycle turning the hexapd right and then left.

<img src="https://github.com/Nabizzle/Hexapod/blob/main/Docs/Media/Hexapod_Turn.gif" width="500">
