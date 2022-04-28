# EMG Controlled 18 DoF Hexapod

![Rendered Hexapod from Fusion360](https://github.com/Nabizzle/Hexapod/blob/main/Docs/Media/Hexapod.PNG)

<p align="center">
  <img src="https://github.com/Nabizzle/Hexapod/blob/main/Docs/Media/walk.gif" width="400" />
  <img src="https://github.com/Nabizzle/Hexapod/blob/main/Docs/Media/turn.gif" width="400" /> 
  <img src="https://github.com/Nabizzle/Hexapod/blob/main/Docs/Media/sit%20and%20stand.gif" width="400" />
</p>

<div align='center'>
  
[![DeepSource](https://deepsource.io/gh/Nabizzle/Hexapod.svg/?label=active+issues&show_trend=true&token=VB3rbYYMZzJr8nYxjxjMuPUo)](https://deepsource.io/gh/Nabizzle/Hexapod/?ref=repository-badge)
[![DeepSource](https://deepsource.io/gh/Nabizzle/Hexapod.svg/?label=resolved+issues&show_trend=true&token=VB3rbYYMZzJr8nYxjxjMuPUo)](https://deepsource.io/gh/Nabizzle/Hexapod/?ref=repository-badge)
[![codecov](https://codecov.io/gh/Nabizzle/Hexapod/branch/main/graph/badge.svg?token=Y0SA932W1L)](https://codecov.io/gh/Nabizzle/Hexapod)
![Twitter Follow](https://img.shields.io/twitter/follow/NabizzleC?style=social)
  
</div>

# Code Coverage Graph

<p align="center">
  <img 
    width="300"
    height="300"
    src="https://codecov.io/gh/Nabizzle/Hexapod/branch/main/graphs/sunburst.svg?token=Y0SA932W1L"
  >
</p>

# Hexapod Project Description

The ultimate goal of this project is to create a Hexapod, or six legged robot, that can move and turn in any direction in the x-y plane and rotate and translate the body of the robot along each axis independent of the legs of the robot. The control of the robot will be from control using two EMG sensors that have their data transmitted to the hexapod or from a phone app. The initial design was adapted from the design of [Capers II](https://www.instructables.com/Capers-II-a-Hexapod-Robot/) recreated in Fusion360 and made to be radially symetrical.

This project is made in two parts. The first is a computational model where the [kinematics calculations](https://github.com/Nabizzle/Hexapod/blob/main/Docs/Kinematics%20Calculations/Hexapod%20Kinematics%20Calculation.pdf) were tested in getting the body to rotate and translate along the x, y, and z axes independent of the leg positions as well as getting the hexapod to walk and turn variable distances. The second part is the onboard code on the raspberry pis that control the physical robot and transmit EMG to the hexapod.

## Getting Started

This code is only the computational model for testing the control of the robot. This is run through the Jupyter Notebook [Walking Model.](https://github.com/Nabizzle/Hexapod/blob/main/Code/Walking%20Model.ipynb) Refer to the notebook to see the outputs of creating the neutral position of an example walk and turn cycle. The bottom setion of the notebook allow you to change the body rotaion and translation before the hexapod walks and turns and the last two cells allow you to change the walking distance and angle as well as the turning angle. This ReadMe corresponds to the Jupyter Notebook. For the library of python functions to run the physical robot, refer to the [library's ReadMe](https://github.com/Nabizzle/Hexapod/tree/main/Code/hexapod)

### Prerequisites

[Python 3.10 or later](https://www.python.org/)
* Required libraries
  * Numpy v1.22.3
  * PySerial v3.5
  * Adafruit Circuit Python MCP3xxx v1.4.7
  * Math
  * Time
  * Typing

### Hexapod Robot
* [Raspberry pi zero WH](https://www.adafruit.com/product/3708) (Original validation testing hardware, but appeared too slow)
* [Raspberry pi 4 Model B (2GB of RAM)](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/)
* 18 MG996R Servos
* [Lynxmotion SSC-32U](https://www.robotshop.com/en/lynxmotion-ssc-32u-usb-servo-controller.html)
* [MCP3008](https://www.adafruit.com/product/856)
* [2 MyoWare v4](https://www.adafruit.com/product/2699)
* [6V 2800mAh Ni-MH Battery](https://www.robotshop.com/en/6v-2800mah-nimh-battery.html)
* [3.7V Lithium Ion Battery](https://www.adafruit.com/product/354)
* [Pimoroni LiPo SHIM](https://www.adafruit.com/product/3196)

## Author

* **Nabeel Chowdhury**

## Acknowledgements

* Toglefritz for his original [hexapod design](https://www.instructables.com/Capers-II-a-Hexapod-Robot/) and [kinematics calculations](https://toglefritz.com/hexapod-inverse-kinematics-equations/) that I adapted for my hexapod
* Oscar Liang for his explanation of the [kinematics equations](https://oscarliang.com/inverse-kinematics-implementation-hexapod-robots/) required.

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

These are functions that relate to creating the model of the hexapod body

### bodyPos

Creates a hexagon model of the body with the given pitch, roll, and yaw angles, the given x, y, and z translations from the center point of the coordinate axes, and the coax servo distance from the center of the coordinate axis.

## Leg Functions

These are functions for finding and refinding the positions of the legs from their servo angles or to do the reverse in finding the servo angles from the end positions of the hexapod feet.

### legPos

This function finds the positions of each segment of the leg from its servo angles, the leg number, and the leg dimensions and then outputs those locations to be added to the larger model of the legs.

### legAngle

Finds the servo angles of the leg based on the x, y, and z positions of the end contact point of the leg and the dimensions of the leg. This function then returns the servo angles as an output.

### recalculateLegAngles

Takes an input of the current body model and current contact points of the legs to calculate the servo angles of all of the legs. This function outputs the servo angles of all of the legs.

### startLegPos

Takes the body model and the starting outer radius the legs make on the robot as wel as the starting height of the robot off of the ground to find the starting servo angles of all of the legs. This function then outputs those angles as an array.

### legModel

Takes in the servo angles of all of the legs as an array and calculates the positions of each leg segement to combine into a model of the legs.

### getFeetPos

This function outputs the contact points of each leg with the ground as x, y, and z coordinates in an array.

## Movement Functions

These are a collection of functions to find how the hexapod will take steps to move linearlly in any direction and to turn itself in the x-y plane.

### stepForward

This function takes in an angle in the x-y plane to move in, a distance to move (in the same units as the model as a whole), a height to move each leg vertically, and a boolean to determine if the "right" legs are moving or the other legs are moving. The "right" legs are legs 0, 2, and 4 or every other leg starting from the front right leg of the robot and moving clockwise around the robot looking from above. This function then outputs the relative distance each foot needs to move compared to where it started to make a step in the desired direction. In order to move, one set of legs will lift up and the other will drag on the ground.

### stepTurnFoot

This function takes the x, y, and z position of a contact point of a leg along with the angle to turn to, the height to move the legs up to, and a boolean to determine if the "right" legs are moving or the other legs are moving. The "right" legs are legs 0, 2, and 4 or every other leg starting from the front right leg of the robot and moving clockwise around the robot looking from above. This funciton then returns the abolute location of each foot. **NOTE** that this is opposed to the relative position in the previous function as each foot can not move by the same relative amount to turn the robot. In order to move, one set of legs will lift up and the other will drag on the ground.

### stepTurn

This funciton takes in the contact positions of all of the legs, the angle to step to, the vertical height of the step, and a boolean to determine if the "right" legs are moving or the other legs are moving. The "right" legs are legs 0, 2, and 4 or every other leg starting from the front right leg of the robot and moving clockwise around the robot looking from above. This function then feeds each foot position into the previous funciton to find the absolute position of each foot.

### walk

Takes the leg model, a linear distance to move, and an angle to move in the x-y plane. This function takes the distance to move and breaks it into steps to move based on a maximum step size. The cycle ends with the hexapod moving to the neutral position. This function requires a positive distance to move. This function is used more as a test of the walking function of the robot. Below is an example of this walk cycle.

<img src="https://github.com/Nabizzle/Hexapod/blob/main/Docs/Media/Hexapod_Walk.gif" width="500">

### turn

Takes the leg model and an angle to turn to. This function breaks the turn into steps based on the maximum turn angle. The cycle ends with the hexapod moving to the neutral position. This function requires a non-zero angle to turn to (positive angles are left turns). This function is used as a test of the turning function. Below is an example of this turn cycle turning the hexapd right and then left.

<img src="https://github.com/Nabizzle/Hexapod/blob/main/Docs/Media/Hexapod_Turn.gif" width="500">

### emgToWalk
Takes the body and leg models, a boolean for which set of feet should move (same as in the previous step functions), the previous step distance, and the maximum distance for a step in the x-y plane. This function polls for two EMG signals to determine the distance to move and scales the maximum step size to the difference between the EMG signals. If the wrist flexor signal and stronger than the wrist extensor signal, the distance to move is positive. If the opposite is true, the distance is negative. The step size is the previous step distance taken plus the new distance. The output of the function is the leg model, the updated foot set boolean after the step, the step size taken during the function to be used in the next run, and the array of leg positions to take a step.

### resetWalkStance
Takes the body and leg models, a boolean for which set of feet should move (same as in the previous step functions), and the previous step distance. This function is meant to reset the stance of the robot when ending the walking cycle before switching to turning. It does this by completing the end of the previous step. The output of the function is the leg model, the updated foot set boolean after the step, and the array of leg positions to take the resetting step.

### emgToTurn
Takes the body and leg models, a boolean for which set of feet should move (same as in the previous step functions), the previous turning angle, and the maximum angle for a turn in the x-y plane. This function polls for two EMG signals to determine the angle to turn and scales the maximum turn angle to the difference between the EMG signals. If the wrist flexor signal and stronger than the wrist extensor signal, the angle to turn is positive (left turn). If the opposite is true, the angle is negative (right turn). The turning step angle is the previous step angle taken plus the new angle. The output of the function is the leg model, the updated foot set boolean after the step, the step angle taken during the function to be used in the next run, and the array of leg positions to take a step.

### resetTurnStance
Takes the body and leg models, a boolean for which set of feet should move (same as in the previous step functions), and the previous turn angle. This function is meant to reset the stance of the robot when ending the turn cycle before switching to walking. It does this by completing the end of the previous turning step. The output of the function is the leg model, the updated foot set boolean after the step, and the array of leg positions to take the resetting step.

### switchMode
Takes in a threshold of EMG to switch the mode. If the user cocontracts above the given threshold, the function returns true to indicate the mode should switch. If the cocontraction is not above threshold, the function returns false.

## Rapsberry Pi to Raspberry Pi Communication
Scripts used in the communications with and between Raspberry Pis.

### pollEMG
Querrys the Raspberry Pi Zero W for the wrist flexor and wrist extensor EMG values. The EMG values are constrained between 0 and 1 and the EMG values are output as a two value list with the flexor and extensor values respectivly.

## Lynxmotiohn SSC-32U Driver
Functions to communicate with the Lynxmotion SSC-32U to drive the 18 servos of the hexapod

### angleToPW
Takes in an angle in degrees and finds the pulse width of the pulse pulse width modulation signal

### anglesToSerial
Takes in a 6x3 numpy array for the 18 servos in the hexapod, the speed the servos should move, and the total time the movement should happen in. The function fomats the angles as a group command and outputs the command as a string.

### connect
Takes in the COM port to connect to and opens the serial port on that COM port.

### disconnect
Disconnects from a the inputted serial port.

### sendData
Takes in a serial port for control and a message to send. Sends the command over the serial port.

## Controller Scripts
Scripts for directly controlling the hexapod aggregating the total work of the full library.

### controller
Connects to the serial port of the Lynxmotiohn SSC-32U, and sets up the starting position of the hexapod. Then the function starts in the walking mode and starts moving based on the EMG signals. After each step, the function checks if the user is cocontracting to switch modes. If they are, the hexapod finishes the step to reset the stance and switches modes.

### sit
Tells the hexapod to sit with its body on the ground.

### stand
Tells the hexapod to move its legs out about the ground and then stand up

<img src="https://github.com/Nabizzle/Hexapod/blob/main/Docs/Media/sit%20and%20stand.gif" width="700">

### walkCycle
The hexapod walks the input distance in the input angle direction without the need for EMG.

<img src="https://github.com/Nabizzle/Hexapod/blob/main/Docs/Media/walk.gif" width="700">

### turnCycle
The hexapod turns the input angle without the need for EMG.

<img src="https://github.com/Nabizzle/Hexapod/blob/main/Docs/Media/turn.gif" width="700">

### sendPostions
Takes in the serial port, the positions to move, and the body model. The function iterates through the array of positions to move and sequentially sends them to the Lynxmotiohn SSC-32U. After each message is sent, there is a pause.
