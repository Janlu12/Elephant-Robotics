import cv2
import numpy as np
from pymycobot.mycobot import MyCobot  # Import knižnice pre robotické rameno

# Funkcia pre detekciu farby
def detect_color(hsv, lower_bound, upper_bound):
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    return mask

# Inicializácia kamery
cap = cv2.VideoCapture(0)

# Inicializácia robotického ramena
# Pripojenie ramena k PC pomocou USB portu
#robot_arm = MyCobot('/dev/ttyUSB0', 115200)  # Príklad pripojenia cez USB port OS Linux

robot_arm = MyCobot("COM3", 115200) #Príklad pripojenia cez USB port OS Windows

# Funkcia pre pohyb ramena
def move_robot_to_sort(color):
    if color == 'red':
        # Nastavte súradnice pre červenú kocku
        robot_arm.send_coords([100, 200, 50, 0, 0, 0], 80, 0)  # Príklad súradníc a rýchlosti
    elif color == 'green':
        # Nastavte súradnice pre zelenú kocku
        robot_arm.send_coords([150, 250, 50, 0, 0, 0], 80, 0)  # Príklad súradníc a rýchlosti
    elif color == 'blue':
        # Nastavte súradnice pre modrú kocku
        robot_arm.send_coords([200, 300, 50, 0, 0, 0], 80, 0)  # Príklad súradníc a rýchlosti

while True:
    # Načítanie snímky z kamery
    ret, frame = cap.read()
    
    if not ret:
        break
    
    # Prevod do HSV farebného priestoru
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Definovanie rozsahu farieb pre detekciu (napr. červená, zelená, modrá)
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask_red = detect_color(hsv, lower_red, upper_red)
    
    lower_green = np.array([36, 100, 100])
    upper_green = np.array([86, 255, 255])
    mask_green = detect_color(hsv, lower_green, upper_green)
    
    lower_blue = np.array([94, 80, 2])
    upper_blue = np.array([126, 255, 255])
    mask_blue = detect_color(hsv, lower_blue, upper_blue)
    
    # Detekcia kontúr
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Spracovanie červených kociek
    for contour in contours_red:
        if cv2.contourArea(contour) > 500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            move_robot_to_sort('red')  #  Kód pre pohyb ramena a triedenie červených kociek
    
    # Spracovanie zelených kociek
    for contour in contours_green:
        if cv2.contourArea(contour) > 500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            move_robot_to_sort('green')  # Kód pre pohyb ramena a triedenie zelených kociek
    
    # Spracovanie modrých kociek
    for contour in contours_blue:
        if cv2.contourArea(contour) > 500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            move_robot_to_sort('blue')  # Kód pre pohyb ramena a triedenie modrých kociek
    
    # Zobrazenie snímky
    cv2.imshow('Frame', frame)
    
    # Ukončenie pomocou klávesy 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Uvoľnenie zdrojov
cap.release()
cv2.destroyAllWindows()
