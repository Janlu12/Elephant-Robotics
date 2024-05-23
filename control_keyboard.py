from elephant_sdk import ElephantSDK
import keyboard  # Make sure to install this module using `pip install keyboard`

class ElephantRobotController:
    """Class for controlling robot from Elephant Robotics."""

    def __init__(self):
        """Initialize instance of robot and SDK."""
        self.robot = ElephantSDK()

    def move_forward(self, distance):
        """Move robot forward to desired units."""
        self.robot.move_forward(distance)
        print(f"Robot moved forward {distance} units.")

    def move_backward(self, distance):
        """Move robot backwards to desired units."""
        self.robot.move_backward(distance)
        print(f"Robot moved backwards {distance} units.")

    def move_left(self, distance):
        """Move robot left to desired units."""
        self.robot.move_left(distance)
        print(f"Robot moved left {distance} units.")

    def move_right(self, distance):
        """Move robot right to desired units."""
        self.robot.move_right(distance)
        print(f"Robot moved right {distance} units.")

    def control_with_keyboard(self):
        """Control robot using WASD keys."""
        distance = 1  # Define the movement distance for each key press

        print("Control the robot using WASD keys. Press 'q' to quit.")

        while True:
            if keyboard.is_pressed('w'):
                self.move_forward(distance)
            elif keyboard.is_pressed('s'):
                self.move_backward(distance)
            elif keyboard.is_pressed('a'):
                self.move_left(distance)
            elif keyboard.is_pressed('d'):
                self.move_right(distance)
            elif keyboard.is_pressed('q'):
                print("Exiting control.")
                break

# Testing control of the robot
robot_controller = ElephantRobotController()
robot_controller.control_with_keyboard()
