 #I've created a project focused on space tracking and navigation utilizing robotics from Elephant Robotics
#I employ a camera for environment mapping and obstacle detection. 
#My aim is to enable the robot to plan optimal routes and navigate around obstacles based on camera input.
#This project allows me to explore the potential of robotics for autonomous navigation and enhance spatial awareness capabilities.


import cv2
import numpy as np
from queue import PriorityQueue
from elephant_sdk import ElephantSDK

class ElephantRobotNavigation:
    def __init__(self, map_width, map_height, robot_ip):
        self.robot = ElephantSDK(robot_ip)
        self.robot.connect()  # Assuming there's a connect method to initialize the connection
        self.map = np.zeros((map_height, map_width, 3), dtype=np.uint8)
        self.robot_position = (map_width // 2, map_height // 2)
        self.obstacle_positions = []

    def update_map(self):
        self.map = np.zeros_like(self.map)
        cv2.circle(self.map, self.robot_position, 5, (0, 255, 0), -1)  # Robot position
        for pos in self.obstacle_positions:
            cv2.circle(self.map, pos, 5, (0, 0, 255), -1)  # Obstacle positions

    def detect_obstacles(self, frame):
        # Detect obstacles in the camera image using computer vision
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.obstacle_positions = [tuple(c[0][0]) for c in contours if cv2.contourArea(c) > 100]

    def plan_path(self, goal_position):
        # Path planning using the A* algorithm
        def heuristic(a, b):
            return np.linalg.norm(np.array(b) - np.array(a))

        start = self.robot_position
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {pos: float('inf') for pos in np.ndindex(self.map.shape[:2])}
        g_score[start] = 0

        while not open_set.empty():
            current = open_set.get()[1]
            if current == goal_position:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                return path[::-1]

            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                next_pos = (current[0] + dx, current[1] + dy)
                if 0 <= next_pos[0] < self.map.shape[1] and 0 <= next_pos[1] < self.map.shape[0]:
                    if next_pos in self.obstacle_positions:
                        continue
                    tentative_g_score = g_score[current] + 1
                    if tentative_g_score < g_score[next_pos]:
                        came_from[next_pos] = current
                        g_score[next_pos] = tentative_g_score
                        f_score = tentative_g_score + heuristic(next_pos, goal_position)
                        open_set.put((f_score, next_pos))
        return None

    def move_robot(self, target_position):
        direction = np.array(target_position) - np.array(self.robot_position)
        distance = np.linalg.norm(direction)
        step_size = 5
        if distance > step_size:
            direction = (direction / distance) * step_size
            new_position = tuple(np.round(np.array(self.robot_position) + direction).astype(int))
            self.robot_position = new_position
            self.robot.move_to(new_position[0], new_position[1])  # Replace with actual SDK command
        else:
            self.robot_position = target_position
            self.robot.move_to(target_position[0], target_position[1])  # Replace with actual SDK command

    def run_navigation(self):
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Unable to read frame from camera.")
                break

            self.detect_obstacles(frame)
            self.update_map()

            goal_position = (self.map.shape[1] - 50, self.map.shape[0] - 50)
            path = self.plan_path(goal_position)

            if path:
                target_position = path[0]
                self.move_robot(target_position)

            cv2.imshow("Navigation Map", self.map)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

# Initialization and running the navigation
robot_ip = "192.168.1.2"  # Replace with the actual IP address of your Elephant Robotics robot
navigation_system = ElephantRobotNavigation(map_width=500, map_height=500, robot_ip=robot_ip)
navigation_system.run_navigation()
