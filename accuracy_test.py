import time
import math

# Import Elephant Robotics SDK (adjust according to the actual library)
from elephant_robotics_sdk import Robot

class MyRobot(Robot):
    def __init__(self, ip_address):
        super().__init__(ip_address)
        self.connect()

    def move_to(self, target_x, target_y):
        # Move to the specified coordinates
        # Ensure this method corresponds to the actual SDK command
        self.move_absolute(target_x, target_y, 0)
        print(f"Moving to: ({target_x}, {target_y})")

    def get_position(self):
        # Return the current position of the robot
        # Ensure this method corresponds to the actual SDK command
        pos = self.get_cartesian_position()
        return pos['x'], pos['y']

def calculate_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def test_robot_accuracy(robot, target_x, target_y, repetitions=100):
    positions = []

    for _ in range(repetitions):
        robot.move_to(target_x, target_y)
        time.sleep(1)  # Wait for the movement to complete (adjust if necessary)
        positions.append(robot.get_position())
        time.sleep(0.1)  # Allow slight delay before next move

    avg_x = sum(pos[0] for pos in positions) / repetitions
    avg_y = sum(pos[1] for pos in positions) / repetitions

    accuracy = calculate_distance((avg_x, avg_y), (target_x, target_y))
    repeatability = sum(calculate_distance(pos, (avg_x, avg_y)) for pos in positions) / repetitions

    return accuracy, repeatability

# Example usage
if __name__ == "__main__":
    ip_address = "192.168.1.1"  # Robot's IP address (adjust according to your setup)
    try:
        robot = MyRobot(ip_address)
        target_x, target_y = 500.0, 500.0  # Target coordinates in mm (adjust according to the robot's specifications)
        repetitions = 100

        accuracy, repeatability = test_robot_accuracy(robot, target_x, target_y, repetitions)

        print(f"Accuracy: {accuracy:.4f} mm")
        print(f"Repeatability: {repeatability:.4f} mm")
    except Exception as e:
        print(f"An error occurred: {e}")
