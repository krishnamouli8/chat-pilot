from dronekit import connect, VehicleMode, LocationGlobalRelative
from rplidar import RPLidar
import math
import time
import threading
import serial
import json
import paho.mqtt.client as mqtt # Import MQTT library
import pyttsx3
from playaudio import play_sound
from AStarSearch import a_star

# =================== CONSTANTS ===================
LIDAR_PORT = '/dev/ttyUSB0'     # Lidar port
PIXHAWK_PORT = '/dev/ttyACM0'   # Pixhawk serial port
# PIXHAWK_PORT = "tcp:192.168.80.1:5762"
DDSM_PORT = '/dev/ttyACM1'
SERIAL_BAUDRATE = 115200
BAUDRATE = 57600
MIN_DISTANCE = 500  # mm (obstacle avoidance threshold)
ALTITUDE = 0.0    
WAYPOINT_REACHED_RADIUS = 1      # For rovers, altitude is 0
FILE_NAME = "cords.txt" # Changed to cords.txt
TURN_SPEED = 30
FORWARD_SPEED = 40
BACKWARD_SPEED = -40
forward_sound = "moveforward.wav"
stop_sound = "stop.wav"
left_sound = "left.wav"
right_sound = "right.wav"

vehicle = None
arrived = False
latest_scan = None  # Global Lidar scan storage
latest_servo1_value = None
latest_servo3_value = None
path = []
location_map = {} # Dictionary to store named locations
engine = None

ddsm_ser = serial.Serial(DDSM_PORT, baudrate=SERIAL_BAUDRATE)
ddsm_ser.setRTS(False)
ddsm_ser.setDTR(False)
print("[System] DDSM Connected")

# =================== MQTT Callbacks ===================
def on_connect(client, userdata, flags, rc):
    print(f"MQTT_CLIENT::Connected with result code {rc}")
    client.subscribe("chatpilot/rover/command") # Subscribe to the command topic

def on_message(client, userdata, msg):
    global path, vehicle
    command_str = msg.payload.decode()
    command_str = command_str.upper()
    print(f"MQTT_CLIENT::Received command: {command_str}")

    # if command_str.startswith("NAVIGATE:"):
    #     # Parse navigation command: NAVIGATE:A,C
    #     parts = command_str.split(":")[1].split(",")
    #     if len(parts) == 2:
    #         start_loc_name = parts[0].strip()
    #         end_loc_name = parts[1].strip()

    #         if start_loc_name in location_map and end_loc_name in location_map:
    #             # For simplicity, let's assume a direct path for now.
    #             # In a real scenario, you'd calculate a path between these points.
    #             path = [location_map[start_loc_name], location_map[end_loc_name]]
    #             print(f"[System] Navigation command received: from {start_loc_name} to {end_loc_name}")
    #             vehicle.simple_goto()
    #             # You might want to trigger the navigation loop here or set a flag
    #             # For this example, we'll let the main loop pick it up.
    #         else:
    #             print(f"[Error] Unknown location names: {start_loc_name} or {end_loc_name}")
    #     else:
    #         print(f"[Error] Invalid NAVIGATE command format: {command_str}")
    if command_str == "FORWARD":
        print("[System] Received command: FORWARD")
        path = [] # Clear path to prioritize manual control
        play_sound(forward_sound)
        motor_control(FORWARD_SPEED, FORWARD_SPEED)
    elif command_str == "LEFT":
        print("[System] Received command: LEFT")
        path = [] # Clear path to prioritize manual control
        play_sound(left_sound)
        motor_control(30, 40) # Negative left speed for turning left
    elif command_str == "RIGHT":
        print("[System] Received command: RIGHT")
        path = [] # Clear path to prioritize manual control
        play_sound(right_sound)
        motor_control(40, 30) # Negative right speed for turning right
    elif command_str == "STOP":
        print("[System] Received command: STOP")
        play_sound(stop_sound)
        path = [] # Clear path to prioritize manual control
        motor_control(0, 0)
    elif command_str == "BACKWARD":
        # Handle other commands if necessary
        #print(f"[System] Other command received: {command_str}")
        print("[System] Command received: BACKWARD")
        motor_control(BACKWARD_SPEED, BACKWARD_SPEED)
    elif command_str.startswith("NAVIGATE"):
        print(f"{command_str}")
        # Navigate: A B
        start = command_str[10]
        end = command_str[12]
        path = a_star(start, end)
        engine.say(f"Navigating from {start} to {end}")
        # engine.runAndWait()
        # engine.stop()
        try:
            for x in path:
                target_location = LocationGlobalRelative(x["coords"][0], x["coords"][1], ALTITUDE)
                goto_position(vehicle, target_location)
                print(f"[System] Reached waypoint {x["node"]+1} --> {x[0]}, {x[1]}")
                time.sleep(0.5)
            print("[System] Reached destination")
            motor_control(0,0)

        except KeyboardInterrupt:
            print("[System] Stopping test...")
            motor_control(0,0)

# =================== FUNCTIONS ===================


def get_haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates in meters"""
    R = 6371000  # Radius of Earth in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)

    a = math.sin(d_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c

def lidar_thread_func(lidar):
    """Lidar scanning in a background thread"""
    global latest_scan
    for scan in lidar.iter_scans():
        latest_scan = scan

def is_front_clear():
    global latest_scan
    scan_data = latest_scan
    if scan_data is None:
        return True # Assume clear if no lidar data
    for (_, angle, dist) in scan_data:
        if (angle >= 340 or angle <= 20) and dist < MIN_DISTANCE and dist > 0:
            return False
    return True

def is_left_clear():
    global latest_scan
    scan_data = latest_scan
    if scan_data is None:
        return True # Assume clear if no lidar data
    for (_, angle, dist) in scan_data:
        if (angle >= 270 or angle <= 340) and dist < MIN_DISTANCE+100 and dist > 0:
            return False
    return True

def scale_servo_to_speed(servo_value):
    if servo_value is None:
        return 0
    # Map servo PWM (1000-2000 µs) to speed (-100 to 100)
    return int((servo_value - 1500) / 500 * 100)

def avoid_obstacle():   
    """Perform an obstacle avoidance maneuver"""
    motor_control(0,0) # stop 
    time.sleep(0.3)
    while not is_front_clear():
        print("[OBSTACLE DETECTED] Avoiding...")
        motor_control(20,-20) # turn right
        time.sleep(0.5)

    motor_control(20 , 20) # move forward
    time.sleep(1.5)

def follow_obstacle():
    while True:
        if not is_front_clear():
            motor_control(TURN_SPEED, -TURN_SPEED)
            print("[OBSTACLE DETECTED] -> turn right")
        elif not is_left_clear():
            motor_control(FORWARD_SPEED, FORWARD_SPEED)
            print("[OBSTACLE DETECTED] -> move forward")
        else:
            break

def goto_position(vehicle, target_location):
    global  latest_servo1_value, latest_servo3_value
    print(f"[Navigation] Moving to target: {target_location.lat}, {target_location.lon}")
    vehicle.simple_goto(target_location)

    while True:
        current_location = vehicle.location.global_relative_frame
        dist_to_target = get_haversine_distance(current_location.lat, current_location.lon , target_location.lat, target_location.lon)
        print(f"[Navigation] Distance to target: {dist_to_target:.2f} meters")

        if dist_to_target <= WAYPOINT_REACHED_RADIUS : # Arrived
            print("[Navigation] Target Reached!")
            break

        if  not is_front_clear():
            print("[Warning] Obstacle detected ahead!")
            follow_obstacle()
            # avoid_obstacle()  # perform avoidance

        servo1 = latest_servo1_value
        servo3 = latest_servo3_value
        speed_left = scale_servo_to_speed(servo1)
        speed_right = scale_servo_to_speed(servo3)
        motor_control(speed_left, speed_right)
        time.sleep(0.1)

def motor_control(left, right):
    global ddsm_ser
    command_right = {
        "T": 10010,
        "id": 2,
        "cmd": -right,  # reverse polarity for right wheel
        "act": 3
    }
    command_left = {
        "T": 10010,
        "id": 1,
        "cmd": left,
        "act": 3
    }
    ddsm_ser.write((json.dumps(command_right) + '\n').encode())
    time.sleep(0.01)
    ddsm_ser.write((json.dumps(command_left) + '\n').encode())

# =================== MAIN ===================

def main():
    global latest_scan, path, location_map, vehicle, engine

    # Initialize MQTT Client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("13.232.191.178", 1883, 60) # Connect to public broker
    client.loop_start() # Start MQTT loop in background thread
    engine = pyttsx3.init()

    # read_coordinates_from_file(FILE_NAME) # Populate location_map
    print(f"[System] Loaded named locations: {location_map.keys()}")

    # print("[System] Starting LIDAR...")
    # lidar = RPLidar(LIDAR_PORT)
    # threading.Thread(target=lidar_thread_func, args=(lidar,), daemon=True).start()
    # while True:
    #     if latest_scan is None:
    #         print("Waiting for LIDAR data...")
    #         time.sleep(1)
    #         continue 
    #     else:
    #         print("Lidar started...")
    #         break

    print("[System] Connecting to Pixhawk...")
    vehicle = connect(PIXHAWK_PORT, baud=BAUDRATE, wait_ready=False)
    print("[System] Connection success...")
    # threading.Thread(target=input_listener, daemon=True).start()

    @vehicle.on_message('SERVO_OUTPUT_RAW')
    def servo_listener(self, name, message):
        global latest_servo1_value, latest_servo3_value
        latest_servo1_value = message.servo1_raw
        latest_servo3_value = message.servo3_raw
        # print(f"[SERVO] Servo1: {latest_servo1_value}, Servo3: {latest_servo3_value}")
          
    print("[System] Arming vehicle...")
    vehicle.armed = True
    while not vehicle.armed:
        print("[System] Waiting for arming...")
        time.sleep(1)
    print("[System] Vehicle Armed.")

    print("[System] Setting GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    # while vehicle.mode.name != "GUIDED":
    #     print("[System] Waiting for GUIDED mode...")
    #     time.sleep(1)
    print(f"[System] Vehicle Mode --> {vehicle.mode.name}")

    try:
        # Main loop to continuously check for navigation commands or execute existing path
        while True:
            if path: # If a path is set (e.g., from an MQTT command)
                # Print the path being traveled
                print(f"[System] Traversing path: {path}") 

                for index, x in enumerate(path):
                    target_location = LocationGlobalRelative(x[0], x[1], ALTITUDE)
                    goto_position(vehicle, target_location)
                    print(f"[System] Reached waypoint {index+1} --> {x[0]}, {x[1]}")
                    time.sleep(0.5)
                print("[System] Reached destination")
                motor_control(0,0)
                path = [] # Clear the path after completion
            time.sleep(1) # Small delay to prevent busy-waiting

    except KeyboardInterrupt:
        print("[System] Stopping test...")
        motor_control(0,0)

    finally:
        vehicle.channels.overrides = {}
        vehicle.armed = False
        vehicle.close()
        # lidar.stop()
        # lidar.stop_motor()
        # lidar.disconnect()
        # ddsm_ser.close()
        client.loop_stop() # Stop MQTT loop
        client.disconnect()

if __name__ == "__main__":
    main()
