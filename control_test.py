from elephant_sdk import ElephantSDK

class ElephantRobotController:
    """Class for controling robot from  Elephant Robotics."""

    def __init__(self):
        """Inicialize instance of robot and SDK."""
        self.robot = ElephantSDK()

    def move_forward(self, distance):
        """Move robot forward to desired units"""
        self.robot.move_forward(distance)
        print(f"Robot moved forward {distance} units.")

    def move_backward(self, distance):
        """Move robot backwards to desired units"""
        self.robot.move_backward(distance)
        print(f"Robot moved backwards in {distance} units.")

# Teting control of robot
robot_controller = ElephantRobotController()
robot_controller.move_forward(10)
robot_controller.move_backward(5)
