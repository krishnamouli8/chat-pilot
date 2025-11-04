from dronekit import connect, VehicleMode, LocationGlobalRelative
import math
import time
import serial
import json
import paho.mqtt.client as mqtt
import pyttsx3
from AStarSearch import a_star

# =================== CONSTANTS ===================
PIXHAWK_PORT = '/dev/ttyACM0'
DDSM_PORT = '/dev/ttyACM1'
SERIAL_BAUDRATE = 115200
BAUDRATE = 57600
ALTITUDE = 0.0
WAYPOINT_REACHED_RADIUS = 1
FORWARD_SPEED = 40
BACKWARD_SPEED = -40

vehicle = None
latest_servo1_value = None
latest_servo3_value = None
path = []
path_total_cost = 0.0
location_map = {}
engine = None

ddsm_ser = serial.Serial(DDSM_PORT, baudrate=SERIAL_BAUDRATE)
ddsm_ser.setRTS(False)
ddsm_ser.setDTR(False)
print("[System] DDSM Connected")

# =================== MQTT CALLBACKS ===================
def on_connect(client, userdata, flags, rc):
    print(f"MQTT_CLIENT::Connected with result code {rc}")
    client.subscribe("chatpilot/rover/command")

def on_message(client, userdata, msg):
    global path, vehicle, path_total_cost
    command_str = msg.payload.decode().strip()
    print(f"MQTT_CLIENT::Received command: {command_str}")

    command_up = command_str.upper()

    if command_up == "FORWARD":
        print("[System] Command: FORWARD")
        path = []
        motor_control(FORWARD_SPEED, FORWARD_SPEED)
    elif command_up == "BACKWARD":
        print("[System] Command: BACKWARD")
        path = []
        motor_control(BACKWARD_SPEED, BACKWARD_SPEED)
    elif command_up == "LEFT":
        print("[System] Command: LEFT")
        path = []
        motor_control(30, 40)
    elif command_up == "RIGHT":
        print("[System] Command: RIGHT")
        path = []
        motor_control(40, 30)
    elif command_up == "STOP":
        print("[System] Command: STOP")
        path = []
        motor_control(0, 0)
    elif command_up.startswith("NAVIGATE"):
        try:
            rest = command_str.split(":", 1)[1]
            parts = [p.strip() for p in rest.split(",")]
            if len(parts) != 2:
                print("[Error] Invalid NAVIGATE format. Use NAVIGATE:START,END")
                return
            start = int(parts[0])
            end = int(parts[1])
        except Exception as e:
            print(f"[Error] Parsing NAVIGATE: {e}")
            return

        # A* search
        found_path, total_cost = a_star(start, end)
        if not found_path:
            print(f"[System] No path found from {start} to {end}.")
            return

        path = found_path
        path_total_cost = total_cost
        print(f"[System] Path with {len(path)} waypoints, cost={total_cost:.2f}m")
        engine.say(f"Navigating from {start} to {end}. Distance {round(total_cost, 2)} meters.")
        engine.runAndWait()

# =================== FUNCTIONS ===================
def get_haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = math.sin(d_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def scale_servo_to_speed(servo_value):
    if servo_value is None:
        return 0
    return int((servo_value - 1500) / 500 * 100)

def goto_position(vehicle, target_location):
    global latest_servo1_value, latest_servo3_value
    if vehicle is None:
        print("[Navigation] No vehicle connected.")
        return

    lat, lon, alt = target_location.lat, target_location.lon, target_location.alt
    print(f"[Navigation] Moving to target: {lat}, {lon}, alt={alt}")
    vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt))

    while True:
        current_location = vehicle.location.global_relative_frame
        if not current_location or not current_location.lat:
            print("[Navigation] Waiting for GPS fix...")
            time.sleep(0.5)
            continue

        dist_to_target = get_haversine_distance(
            current_location.lat, current_location.lon,
            lat, lon
        )
        print(f"[Navigation] Distance to target: {dist_to_target:.2f}m")

        if dist_to_target <= WAYPOINT_REACHED_RADIUS:
            print("[Navigation] Target reached.")
            break

        servo1, servo3 = latest_servo1_value, latest_servo3_value
        speed_left = scale_servo_to_speed(servo1)
        speed_right = scale_servo_to_speed(servo3)
        motor_control(speed_left, speed_right)
        time.sleep(0.1)

def motor_control(left, right):
    global ddsm_ser
    cmd_right = {"T": 10010, "id": 2, "cmd": -right, "act": 3}
    cmd_left = {"T": 10010, "id": 1, "cmd": left, "act": 3}
    ddsm_ser.write((json.dumps(cmd_right) + '\n').encode())
    time.sleep(0.01)
    ddsm_ser.write((json.dumps(cmd_left) + '\n').encode())

# =================== MAIN ===================
def main():
    global path, path_total_cost, location_map, vehicle, engine

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("13.232.191.178", 1883, 60)
    client.loop_start()

    engine = pyttsx3.init()
    print(f"[System] Loaded named locations: {location_map.keys()}")

    print("[System] Connecting to Pixhawk...")
    vehicle = connect(PIXHAWK_PORT, baud=BAUDRATE, wait_ready=False)
    print("[System] Connection success...")

    @vehicle.on_message('SERVO_OUTPUT_RAW')
    def servo_listener(self, name, message):
        global latest_servo1_value, latest_servo3_value
        latest_servo1_value = message.servo1_raw
        latest_servo3_value = message.servo3_raw

    print("[System] Arming vehicle...")
    vehicle.armed = True
    while not vehicle.armed:
        print("[System] Waiting for arming...")
        time.sleep(1)
    print("[System] Vehicle Armed.")

    print("[System] Setting GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    print(f"[System] Vehicle Mode --> {vehicle.mode.name}")

    try:
        while True:
            if path:
                print(f"[System] Traversing path with {len(path)} waypoints (cost: {path_total_cost:.2f}m).")
                for index, node in enumerate(path):
                    lat, lon = node["coords"][0], node["coords"][1]
                    target = LocationGlobalRelative(float(lat), float(lon), ALTITUDE)
                    goto_position(vehicle, target)
                    print(f"[System] Reached waypoint {index+1}: {lat}, {lon}")
                    time.sleep(0.5)
                print("[System] Destination reached.")
                motor_control(0, 0)
                path.clear()
                path_total_cost = 0.0
            time.sleep(1)

    except KeyboardInterrupt:
        print("[System] Keyboard interrupt, stopping...")
        motor_control(0, 0)

    finally:
        try:
            vehicle.channels.overrides = {}
            vehicle.armed = False
            vehicle.close()
        except Exception:
            pass
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()
