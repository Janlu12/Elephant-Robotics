import numpy as np
import cv2
import random
from elephant_sdk import ElephantSDK
from scipy.spatial import KDTree

class ElephantRobotArmNavigation:
    def __init__(self, robot_ip):
        self.robot = ElephantSDK(robot_ip)
        self.robot.connect()
        self.obstacle_positions = []
        self.target_position = None

    def detect_obstacles(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.obstacle_positions = [tuple(c[0][0]) for c in contours if cv2.contourArea(c) > 100]

    def plan_path(self, goal_position):
        # RRT algoritmus
        start = self.robot_position
        goal = goal_position

        def distance(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))

        def is_collision_free(point, obstacle_positions, threshold=10):
            for obs in obstacle_positions:
                if distance(point, obs) < threshold:
                    return False
            return True

        class Node:
            def __init__(self, position, parent=None):
                self.position = position
                self.parent = parent

        max_iter = 1000
        tree = [Node(start)]
        for _ in range(max_iter):
            rand_point = (random.randint(0, self.map.shape[1]), random.randint(0, self.map.shape[0]))
            nearest_node = min(tree, key=lambda node: distance(node.position, rand_point))
            direction = np.array(rand_point) - np.array(nearest_node.position)
            length = np.linalg.norm(direction)
            direction = direction / length
            new_position = tuple(np.round(np.array(nearest_node.position) + direction * 10).astype(int))

            if is_collision_free(new_position, self.obstacle_positions):
                new_node = Node(new_position, nearest_node)
                tree.append(new_node)
                if distance(new_position, goal) < 10:
                    path = []
                    while new_node is not None:
                        path.append(new_node.position)
                        new_node = new_node.parent
                    return path[::-1]
        
        return None

    def move_robot(self, target_position):
        direction = np.array(target_position) - np.array(self.robot_position)
        distance = np.linalg.norm(direction)
        step_size = 5
        if distance > step_size:
            direction = (direction / distance) * step_size
            new_position = tuple(np.round(np.array(self.robot_position) + direction).astype(int))
            self.robot_position = new_position
            self.robot.move_to(new_position[0], new_position[1])
        else:
            self.robot_position = target_position
            self.robot.move_to(target_position[0], target_position[1])

    def run_navigation(self):
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Nemožno načítať záber z kamery.")
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

# Inicializácia a spustenie navigácie
robot_ip = "192.168.1.2"
navigation_system = ElephantRobotArmNavigation(robot_ip=robot_ip)
navigation_system.run_navigation()
