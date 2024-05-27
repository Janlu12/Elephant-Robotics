import numpy as np
import cv2
import random
from pymycobot.mycobot import MyCobot #correct library for MyCobot 
from scipy.spatial import KDTree




mc = MyCobot("COM3", 115200) # Initiate MyCobot

class ElephantRobotArmNavigation:
    def __init__(self, port, baudrate):  #  def __init__(self, robot_ip): 
         self.robot = MyCobot(port, baudrate)
         self.robot.connect()  #
         self.robot_position = (250 , 250)
         self.obstacle_positions = []
         #mc.target_position = None
         self.map = np.zeros((500, 500, 3), dtype=np.uint8)

    def detect_obstacles(mc, frame):
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

def move_arm_to(self, target_position):
        # Pohyb ramena k cieľovej pozícii
        x, y = target_position
        z = 100  # Predpokladaná výška zápästia
        self.send_angles([x, y, z, 0, 0, 0], 80)  # Nastavenie pozície ramena

        time.sleep(2.5) #Cakacia doba na zistenie ci rameno dosiahlo požadovanu poziciu 
    

    def run_navigation(self):
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            if not ret:
                print("CANNOT LOAD CAMERA SCREEN !!")
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
port = "COM3"  # Nahraďte skutočným portom pre váš MyCobot
baudrate = 115200  # Predvolený baudrate pre MyCobot
navigation_system = ElephantRobotArmNavigation(port, baudrate)
navigation_system.run_navigation()
