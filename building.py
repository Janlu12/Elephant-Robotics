''' Building blocks with Elephant Robotics robot'''

from elephant_sdk import ElephantSDK

class ElephantTowerBuilder:
    """Class for building a tower from blocks using an Elephant Robotics robot."""

    def __init__(self, robot_ip):
        """Initialize the tower builder with the robot and set the tower height to 0."""
        self.robot = ElephantSDK(robot_ip)  # Connect to the robot using its IP address
        self.robot.connect()  # Assuming there's a connect method to initialize the connection
        self.tower_height = 0

    def add_block(self):
        """Add a block to the tower and increase its height by 1."""
        # Command the robot to pick up a block and place it on the tower
        self.robot.pick_block()
        self.robot.place_block_on_tower(self.tower_height)
        self.tower_height += 1
        print(f"+1 block added to the tower. Current height of the tower is: {self.tower_height}")

    def remove_block(self):
        """Remove a block from the tower if the tower is higher than 0."""
        if self.tower_height > 0:
            # Command the robot to remove the top block from the tower
            self.robot.remove_block_from_tower(self.tower_height - 1)
            self.tower_height -= 1
            print(f"-1 block taken from the tower. Current height of the tower is: {self.tower_height}")
        else:
            print("The tower is already at ground level.")

# Example usage
robot_ip = "192.168.1.2"  # Replace with the actual IP address of your Elephant Robotics robot
tower_builder = ElephantTowerBuilder(robot_ip)
tower_builder.add_block()
tower_builder.add_block()
tower_builder.remove_block()
