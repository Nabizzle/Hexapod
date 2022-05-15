from inputs import get_gamepad
from math import pow
from threading import Thread
from typing import List

class xboxController():
    """
    A class object to contain all controller analog and state values

    A class object for the xbox one controller that stores all controller
    button and analog statea and values. Also holds values to normalize the
    analog stick and trigger values between -1 and 1.

    Attributes
    ----------
    right_joystick_y: float, default 0
        The analog value in the y direction of the right analog stick
    right_joystick_x: float, default 0
        The analog value in the x direction of the right analog stick
    right_thumb: int, default 0
        The state value of the right analog stick pressed in
    left_joystick_y: float, default 0
        The analog value in the y direction of the left analog stick
    left_joystick_x: float, default 0
        The analog value in the x direction of the leftt analog stick
    left_thumb: int, default 0
        The state value of the leftt analog stick pressed in
    a: int, default 0
        The state of the a button
    b: int, default 0
        The state of the b button
    y: int, default 0
        The state of the y button
    x: int, default 0
        The state of the x button
    down_up_d_pad: int, default 0
        The state of the up down direction of the D pad. Up is -1 and down is 1
    right_left_d_pad: int, default 0
        The state of the right left direction of the D pad. left is -1 and
        right is 1
    right_trigger: float, default 0
        The analog value of the right trigger
    right_bumper: int, default 0
        The state of the right bumper
    left_trigger: float, default 0
        The analog value of the leftt trigger
    left_bumper: int, default 0
        The state of the leftt bumper
    back: int, default 0
        The state of the back button
    start: int, default 0
        The state of the start button
    poll_thread: Thread
        The thread to poll each of the parts of the xbox one controller
    """
    MAX_TRIG_VAL = pow(2, 8) # The maximum value of the triggers
    MAX_JOY_VAL = pow(2, 15) # The maximum value of the joy sticks

    def __init__(self) -> None:
        self.right_joystick_y = 0
        self.right_joystick_x = 0
        self.right_thumb = 0

        self.left_joystick_y = 0
        self.left_joystick_x = 0
        self.left_thumb = 0

        self.a = 0
        self.b = 0
        self.y = 0
        self.x = 0

        self.down_up_d_pad = 0
        self.right_left_d_pad = 0

        self.right_trigger = 0
        self.right_bumper = 0
        self.left_trigger = 0
        self.left_bumper = 0

        self.back = 0
        self.start = 0

        self._poll_thread =\
            Thread(target=self._poll_controller, args=())
        self._poll_thread.daemon = True
        self._poll_thread.start()


    def read(self) -> List:
        """
        Returns the values of all parts of the controller

        Returns the analog values and states of all buttons, joy sticks, and
        triggers of the xbox one controller.

        Parameters
        ----------
        self: xboxController
            The xboxController class object

        Returns
        -------
        rs_y: float, default 0
        The analog value in the y direction of the right analog stick
        rs_x: float, default 0
            The analog value in the x direction of the right analog stick
        rs_t: int, default 0
            The state value of the right analog stick pressed in
        ls_y: float, default 0
            The analog value in the y direction of the left analog stick
        ls_x: float, default 0
            The analog value in the x direction of the leftt analog stick
        ls_t: int, default 0
            The state value of the leftt analog stick pressed in
        a: int, default 0
            The state of the a button
        b: int, default 0
            The state of the b button
        y: int, default 0
            The state of the y button
        x: int, default 0
            The state of the x button
        down_up_d: int, default 0
            The state of the up down direction of the D pad. Up is -1 and down is 1
        right_left_d: int, default 0
            The state of the right left direction of the D pad. left is -1 and
            right is 1
        rt: float, default 0
            The analog value of the right trigger
        rb: int, default 0
            The state of the right bumper
        lt: float, default 0
            The analog value of the leftt trigger
        lb: int, default 0
            The state of the leftt bumper
        back: int, default 0
            The state of the back button
        start: int, default 0
            The state of the start button

        Notes
        -----
        Returns the analog and state values in the order of the atrribute
        declarations in the __init__ funciton.
        """
        rs_x = self.right_joystick_x
        rs_y = self.right_joystick_y
        rs_t = self.right_thumb

        ls_x = self.left_joystick_x
        ls_y = self.left_joystick_y
        ls_t = self.left_thumb

        a = self.a
        b = self.b
        y = self.y
        x = self.x

        down_up_d = self.down_up_d_pad
        right_left_d = self.right_left_d_pad

        rt = self.right_trigger
        rb = self.right_bumper

        lt = self.left_trigger
        lb = self.left_bumper

        back = self.back
        start = self.start
        return [rs_x, rs_y, rs_t, ls_x, ls_y, ls_t, a, b, y, x, down_up_d,
                right_left_d, rt, rb, lt, lb, back, start]


    def _poll_controller(self) -> None:
        """
        Sets all controller values in a thread

        At the time of an event, the thread sets the appropriate
        xboxController atrribute
        """
        while True:
            events = get_gamepad()
            for event in events:
                match event.code:
                    case 'ABS_RX':
                        self.right_joystick_x = event.state / xboxController.MAX_JOY_VAL # normalize between -1 and 1
                    case 'ABS_RY':
                        self.right_joystick_y = event.state / xboxController.MAX_JOY_VAL # normalize between -1 and 1
                    case 'BTN_THUMBR':
                        self.right_thumb = event.state

                    case 'ABS_X':
                        self.left_joystick_x = event.state / xboxController.MAX_JOY_VAL # normalize between -1 and 1
                    case 'ABS_Y':
                        self.left_joystick_y = event.state / xboxController.MAX_JOY_VAL # normalize between -1 and 1
                    case 'BTN_THUMBL':
                        self.left_thumb = event.state

                    case 'BTN_SOUTH':
                        self.a = event.state
                    case 'BTN_EAST':
                        self.b = event.state
                    case 'BTN_NORTH':
                        self.y = event.state
                    case 'BTN_WEST':
                        self.x = event.state

                    case 'ABS_HAT0Y':
                        self.down_up_d_pad = event.state
                    case 'ABS_HAT0X':
                        self.right_left_d_pad = event.state
                    case 'ABS_RZ':
                        self.right_trigger = event.state / xboxController.MAX_TRIG_VAL # normalize between 0 and 1
                    case 'BTN_TR':
                        self.right_bumper = event.state

                    case 'ABS_Z':
                        self.left_trigger = event.state / xboxController.MAX_TRIG_VAL # normalize between 0 and 1
                    case 'BTN_TL':
                        self.left_bumper = event.state

                    case 'BTN_START':
                        self.back = event.state
                    case 'BTN_SELECT':
                        self.start = event.state
                    

if __name__ == '__main__':
    """
    Create a controller and print out all controller values

    See Also
    --------
    hexapod.xbox.read:
        Reads out all the controller values
    """
    controller = xboxController()
    while True:
        print(controller.read())