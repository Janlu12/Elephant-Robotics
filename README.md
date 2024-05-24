# Elephant-Robotics
Elephant Robotics project


Preview of robot representation

**accurracy_of_robot.py**


1. **MyRobot Class**:
    - Inherits from the `Robot` class provided by Elephant Robotics SDK.
    - `__init__`: Connects to the robot using its IP address.
    - `move_to`: Moves the robot to the target coordinates. The `move_absolute` function might be different based on the SDK documentation.
    - `get_position`: Returns the current position of the robot in the format `{ 'x': x_value, 'y': y_value, ... }`.

2. **calculate_distance**: Calculates the distance between two points in 2D space.

3. **test_robot_accuracy**:
    - Moves the robot to the target position multiple times (defined by the `repetitions` parameter).
    - Records the positions after each move.
    - Calculates the average position (to estimate accuracy).
    - Calculates accuracy as the distance between the average position and the target position.
    - Calculates repeatability as the average distance between each recorded position and the average position.

4. **Main Part of the Program**:
    - Creates an instance of the robot and connects to it.
    - Sets the target position and the number of repetitions.
    - Calls the function to test accuracy and repeatability.
    - Prints the results.


**building.py**



1. **Connecting to the Robot**:
    - The `__init__` method now takes `robot_ip` as a parameter to initialize the connection to the robot using its IP address.
    - `self.robot.connect()`: This line assumes there is a method to connect to the robot. Adjust this according to the actual SDK documentation.

2. **Adding a Block**:
    - `self.robot.pick_block()`: Assumes there is a method in the SDK to pick up a block.
    - `self.robot.place_block_on_tower(self.tower_height)`: Assumes there is a method to place a block on top of the current tower height.

3. **Removing a Block**:
    - `self.robot.remove_block_from_tower(self.tower_height - 1)`: Assumes there is a method to remove the top block from the tower.
  

   **control_keaboard.py**


1. **Movement Methods**:
    - Added `move_left` and `move_right` methods to handle lateral movements of the robot.

2. **Keyboard Control Method**:
    - `control_with_keyboard`: This method continuously checks for key presses. When a WASD key is pressed, the corresponding movement method is called. The `distance` variable defines the distance moved per key press.

3. **Main Control Loop**:
    - The control loop continuously listens for key presses using `keyboard.is_pressed` and executes the corresponding movement methods.
    - Pressing 'q' exits the control loop and stops the program.

### Requirements

- Ensure you have the `keyboard` module installed. You can install it using pip:
  ```bash
  pip install keyboard
  ```
- The actual SDK methods for moving the robot (`move_forward`, `move_backward`, `move_left`, `move_right`) should be defined in the `ElephantSDK` class. Adjust the method names and parameters according to the actual SDK documentation if necessary. 

This setup allows you to control the robot in real-time using the keyboard, making it interactive and suitable for quick testing and adjustments.


**mapping.py**

1. **Robot Initialization**:
   - Added `robot_ip` to the constructor to initialize the connection to the robot.
   - `self.robot.connect()`: Connect to the robot using the IP address.

2. **Update Map**:
   - Updated to use `cv2.circle` for marking the robot position and obstacles for better visualization.

3. **Detect Obstacles**:
   - Fixed the tuple extraction from contours for obstacle detection.

4. **Move Robot**:
   - Updated to include `self.robot.move_to(new_position[0], new_position[1])`, assuming `move_to` is the correct method from the SDK for moving the robot to specified coordinates.
   - Adjusted `move_robot` to ensure proper movement commands to the robot.

5. **Camera Handling**:
   - Added `cap.release()` to release the camera resource when the loop ends.


**arm_mapping.py**


This code defines a navigation system for a robot arm from Elephant Robotics. It includes functionalities for detecting obstacles using a camera, updating a visual map, planning a path to a goal position using the RRT (Rapidly-exploring Random Tree) algorithm, and moving the robot arm along the planned path.



1. **Initialization (`__init__` method)**:
   - Connects to the robot arm using its IP address.
   - Initializes the robot's position and the map (a 500x500 pixel grid).
   - Initializes an empty list for obstacle positions.

2. **Obstacle Detection (`detect_obstacles` method)**:
   - Captures a frame from the camera and converts it to grayscale.
   - Applies a binary threshold to the grayscale image.
   - Finds contours in the binary image.
   - Extracts the positions of contours that are large enough to be considered obstacles.

3. **Map Update (`update_map` method)**:
   - Resets the map to an empty grid.
   - Draws the robot's current position on the map as a green circle.
   - Draws the detected obstacles on the map as red circles.

4. **Path Planning (`plan_path` method)**:
   - Uses the RRT algorithm to find a path from the robot's current position to the goal position.
   - Defines a `Node` class to represent points in the tree with their positions and parent nodes.
   - Repeatedly generates random points and extends the tree towards these points.
   - Checks for collisions with obstacles and ensures the new points are collision-free.
   - If a point close enough to the goal is found, it constructs and returns the path from start to goal by backtracking through the parent nodes.

5. **Robot Movement (`move_robot` method)**:
   - Calculates the direction and distance to the target position.
   - Moves the robot in small steps towards the target position.
   - Updates the robot's position and sends move commands to the robot arm.

6. **Navigation Loop (`run_navigation` method)**:
   - Continuously captures frames from the camera.
   - Detects obstacles in the current frame.
   - Updates the visual map.
   - Plans a path to a predefined goal position (near the bottom-right corner of the map).
   - Moves the robot along the planned path, step by step.
   - Displays the map with the robot's position and obstacles.
   - Stops the loop when the 'q' key is pressed.

7. **Main Execution**:
   - Initializes the navigation system with the robot's IP address.
   - Starts the navigation loop.

### Key Points of the Code:

- **Obstacle Detection**: Uses computer vision techniques to identify obstacles in the robot's environment.
- **Path Planning**: Implements the RRT algorithm to find a collision-free path to the goal.
- **Robot Control**: Moves the robot towards the goal position using calculated steps.
- **Real-time Visualization**: Continuously updates and displays the robot's map with its position and detected obstacles.


