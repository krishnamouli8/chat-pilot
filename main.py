from dronekit import connect, VehicleMode, LocationGlobalRelative
from rplidar import RPLidar
from pymavlink import mavutil
import math
import time
import threading
import json
import paho.mqtt.client as mqtt

# =================== CONSTANTS ===================
PIXHAWK_PORT = "tcp:192.168.80.1:5762"
BAUDRATE = 57600
MIN_DISTANCE = 500  # mm obstacle threshold
ALTITUDE = 0.0    
WAYPOINT_REACHED_RADIUS = 1
FILE_NAME = "cords.txt"

# Manual control variables
manual_mode = False
manual_command = None

# Lidar storage
latest_scan = None
location_map = {}
path = []

# =================== MQTT Callbacks ===================
def on_connect(client, userdata, flags, rc):
    print(f"[MQTT] Connected with result code {rc}")
    client.subscribe("rover/command")

def on_message(client, userdata, msg):
    global path, manual_mode, manual_command
    command_str = msg.payload.decode().strip()
    print(f"[MQTT] Received command: {command_str}")

    if command_str.startswith("NAVIGATE:"):
        parts = command_str.split(":")[1].split(",")
        if len(parts) == 2:
            start = parts[0].strip()
            end = parts[1].strip()
            if start in location_map and end in location_map:
                path = [location_map[start], location_map[end]]
                manual_mode = False
                manual_command = None
            else:
                print(f"[Error] Unknown locations: {start}, {end}")
        else:
            print(f"[Error] Invalid NAVIGATE command: {command_str}")

    elif command_str in ["FORWARD", "LEFT", "RIGHT"]:
        manual_mode = True
        manual_command = command_str

    elif command_str == "STOP":
        manual_mode = False
        manual_command = None

# =================== FUNCTIONS ===================
def read_coordinates_from_file(filename):
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip().replace(" ", "")
            if not line:
                continue
            try:
                name, coords = line.split(':')
                lat, lon = coords.split(',')[:2]
                location_map[name] = (float(lat), float(lon))
            except Exception as e:
                print(f"[Error] Skipping line: {line} ({e})")

def get_haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = math.sin(d_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def lidar_thread_func(lidar):
    global latest_scan
    for scan in lidar.iter_scans():
        latest_scan = scan

def is_front_clear():
    if latest_scan is None:
        return True
    for (_, angle, dist) in latest_scan:
        if (angle >= 340 or angle <= 20) and dist < MIN_DISTANCE and dist > 0:
            return False
    return True

def send_velocity(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # only velocity
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def goto_position(vehicle, target_location):
    print(f"[Nav] Going to {target_location.lat}, {target_location.lon}")
    vehicle.simple_goto(target_location)
    while True:
        cur = vehicle.location.global_relative_frame
        dist = get_haversine_distance(cur.lat, cur.lon, target_location.lat, target_location.lon)
        print(f"[Nav] Distance: {dist:.2f} m")
        if dist <= WAYPOINT_REACHED_RADIUS:
            print("[Nav] Target reached.")
            break
        if not is_front_clear():
            print("[Obstacle] Stopping to avoid collision.")
            send_velocity(vehicle, 0, 0, 0)
            while not is_front_clear():
                time.sleep(0.1)
            print("[Obstacle] Path clear. Resuming.")
            vehicle.simple_goto(target_location)
        time.sleep(0.1)

# =================== MAIN ===================
def main():
    global manual_mode, manual_command, path

    # MQTT Setup
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("test.mosquitto.org", 1883, 60)
    client.loop_start()

    read_coordinates_from_file(FILE_NAME)
    print(f"[System] Locations loaded: {location_map.keys()}")

    print("[System] Connecting to Pixhawk...")
    vehicle = connect(PIXHAWK_PORT, baud=BAUDRATE, wait_ready=False)
    print("[System] Connected.")

    print("[System] Arming...")
    vehicle.armed = True
    while not vehicle.armed:
        print("[System] Waiting for arming...")
        time.sleep(1)

    print("[System] Setting GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(1)

    try:
        while True:
            if manual_mode:
                if manual_command == "FORWARD":
                    send_velocity(vehicle, 1.0, 0, 0)
                elif manual_command == "LEFT":
                    send_velocity(vehicle, 0, -0.5, 0)
                elif manual_command == "RIGHT":
                    send_velocity(vehicle, 0, 0.5, 0)
            else:
                send_velocity(vehicle, 0, 0, 0)  # stop when not in manual mode
                if path:
                    for lat, lon in path:
                        target = LocationGlobalRelative(lat, lon, ALTITUDE)
                        goto_position(vehicle, target)
                    path = []

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("[System] Interrupted. Stopping...")
        send_velocity(vehicle, 0, 0, 0)

    finally:
        vehicle.close()
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()
