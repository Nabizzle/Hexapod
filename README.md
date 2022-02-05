# Table of Contents

* [Hexapod Project Description](#hexapod-project-description)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
* [Author](#author)
* [Acknowledgements](#acknowledgements)
* [Description of Each Script](#description-of-each-script)
  * [Rotation Matrices](#rotation-matrices)
    * [xRot](#xrot)
    * [yRot](#yrot)
    * [zRot](#zrot)
  * [Body Functions](#body-functions)
    * [bodyPos](#bodypos)
  * [Leg Functions](#leg-functions)
    * [legPos](#legpos)
    * [legAngle](#legangle)
    * [startLegPos](#startlegpos)
    * [getFeetPos](#getfeetpos)
    * [recalculateLegAngles](#recalculatelegangles)
    * [legModel](#legmodel)
  * [Movement Functions](#movement-functions)
    * [stepForward](#stepforward)
    * [stepTurnFoot](#stepturnfoot)
    * [stepTurn](#stepturn)
  * [Movement Cycles](#movement-cycles)
    * [walk](#walk)
    * [turn](#turn)
  * [Model Visualization](#model-visualization)
    * [showModel](#showmodel)
    * [animate](#animate)
    * [animateBodyTranslate](#animatebodytranslate)

# Hexapod Project Description

The ultimate goal of this project is to create a Hexapod, or six legged robot, that can move and turn in any direction in the x-y plane and rotate and translate the body of the robot along each axis independent of the legs of the robot. The control of the robot will be from control using two EMG sensors that have their data transmitted to the hexapod or from a phone app. The initial design was adapted from the design of [Capers II](https://www.instructables.com/Capers-II-a-Hexapod-Robot/) recreated in Fusion360 and made to be radially symetrical.

This project is made in two parts. The first is a computational model where the [kinematics calculations](https://github.com/Nabizzle/Hexapod/blob/main/Docs/Kinematics%20Calculations/Hexapod%20Kinematics%20Calculation.pdf)
![Rendered Hexapod from Fusion360](https://github.com/Nabizzle/Hexapod/blob/main/Docs/Media/Hexapod.PNG) were tested in getting the body to rotate and translate along the x, y, and z axes independent of the leg positions as well as getting the hexapod to walk and turn variable distances. The second part, which is yet to be made, is the onboard code on the raspberry pis that control the physical robot and transmit EMG to the hexapod.

## Getting Started

The current sete of code is only the computational model for testing the control of the robot. This is run through the Jupyter Notebook [Walking Model.](https://github.com/Nabizzle/Hexapod/blob/main/Code/Walking%20Model.ipynb) Refer to the notebook to see the outputs of creating the neutral position of an example walk and turn cycle. The bottom setion of the notebook allow you to change the body rotaion and translation before the hexapod walks and turns and the last two cells allow you to change the walking distance and angle as well as the turning angle.

### Prerequisites

[Python 3.8.12 or later](https://www.python.org/)
* Required libraries
  * Numpy
  * Matplotlib
  * Seaborn
  * Math
  * Time

Hexapod Robot
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

Takes the body and leg models, a figure to plot on, a linear distance to move, and an angle to move in in the x-y frome. This function takes the distance to move and breaks it into steps to move based on a maximum step size. The cycle ends with the hexapod moving to the neutral position. This function requires a positive distance to move and ends with the creation of a video, saved to the media folder, of the movement. Below is an example of this walk cycle.

<img src="https://github.com/Nabizzle/Hexapod/blob/main/Docs/Media/Hexapod_Walk.gif" width="500">

### turn

Takes the body and leg models, a figure to plot on, and an angle to turn to. This function breaks the turn into steps based on the maximum turn angle. The cycle ends with the hexapod moving to the neutral position. This function requires a non-zero angle to turn to (positive angles are left turns) and ends with the creation of a video, saved to the media folder, of the movement. Left and right turns are saved as seperate videos. Below is an example of this turn cycle turning the hexapd right and then left.

<img src="https://github.com/Nabizzle/Hexapod/blob/main/Docs/Media/Hexapod_Turn.gif" width="500">

## Model Visualization

Functions to plot the full body and leg model and take in an array of leg movements to animate the walk and turn cycles

### showModel

Takes the body and leg models, the figure to plot on, the floor height, and an elevation and azmuth angle to plot a 3D figure at. This funciton draws the body as green, the "right" legs as blue, and the "left" legs as red.

### animate

This is the function required by the movie writer to create a video. This takes in the same parameters as the showModel function as well as the leg movements to animate the walk and turn cycles.

### animateBodyTranslate

This function takes a specific frame number and figure to iterate through translation and rotations arrays in order to create a video of the body of the hexapod moving in all degrees of freedom independent of leg position.

<img src="https://github.com/Nabizzle/Hexapod/blob/main/Docs/Media/Hexapod_Body_Translation.gif" width="500">
