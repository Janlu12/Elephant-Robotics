#This code is for connecting Elephant Robotics robot usig various methods like IP , Serial Port or Bluetooth adress

class MyRobot(Robot):
    def __init__(self, ip_address=None, connection_method="ip", serial_port=None, bluetooth_address=None):
        """
        Initialize the robot with various connection methods.

        :param ip_address: IP address for network connection
        :param connection_method: Method to connect to the robot ("ip", "serial", "bluetooth")
        :param serial_port: Serial port for serial connection
        :param bluetooth_address: Bluetooth address for Bluetooth connection
        """
        super().__init__()
        self.connection_method = connection_method
        self.ip_address = ip_address
        self.serial_port = serial_port
        self.bluetooth_address = bluetooth_address

        self.connect()

    def connect(self):
        """
        Connect to the robot based on the specified connection method.
        """
        if self.connection_method == "ip" and self.ip_address:
            self.connect_via_ip(self.ip_address)
        elif self.connection_method == "serial" and self.serial_port:
            self.connect_via_serial(self.serial_port)
        elif self.connection_method == "bluetooth" and self.bluetooth_address:
            self.connect_via_bluetooth(self.bluetooth_address)
        else:
            raise ValueError("Invalid connection method or missing parameters")

    def connect_via_ip(self, ip_address):
        """
        Connect to the robot via IP address.
        """
        # Replace with the actual SDK command to connect via IP
        self.connect_to_ip(ip_address)
        print(f"Connected to the robot via IP address: {ip_address}")

    def connect_via_serial(self, serial_port):
        """
        Connect to the robot via serial port.
        """
        # Replace with the actual SDK command to connect via serial port
        self.connect_to_serial(serial_port)
        print(f"Connected to the robot via serial port: {serial_port}")

    def connect_via_bluetooth(self, bluetooth_address):
        """
        Connect to the robot via Bluetooth.
        """
        # Replace with the actual SDK command to connect via Bluetooth
        self.connect_to_bluetooth(bluetooth_address)
        print(f"Connected to the robot via Bluetooth address: {bluetooth_address}")

# Example usage
# Connect using IP address
robot_ip = "192.168.1.2"
my_robot_ip = MyRobot(ip_address=robot_ip, connection_method="ip")

# Connect using serial port
serial_port = "/dev/ttyUSB0"
my_robot_serial = MyRobot(serial_port=serial_port, connection_method="serial")

# Connect using Bluetooth
bluetooth_address = "00:1A:7D:DA:71:13"
my_robot_bluetooth = MyRobot(bluetooth_address=bluetooth_address, connection_method="bluetooth")
