from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import json
import paho.mqtt.client as mqtt
# from playaudio import play_sound
import math
import os

# =================== CONSTANTS ===================
PIXHAWK_PORT = 'tcp:127.0.0.1:5763'   # Pixhawk serial port
BAUDRATE = 57600
ALTITUDE = 0.0
WAYPOINT_REACHED_RADIUS = 1.5  # meters
FILE_NAME = "cords.txt"

# Sound files
forward_sound = "moveforward.wav"
stop_sound = "stop.wav"
left_sound = "left.wav"
right_sound = "right.wav"

vehicle = None
path = []
location_map = {}  # Dictionary to store node_name -> (lat, lon)


# =================== READ COORDINATES ===================
def read_coordinates_from_file(filename):
    """Reads space-separated coordinates from txt file and returns dict"""
    coords = {}
    if not os.path.exists(filename):
        print(f"[Error] File not found: {filename}")
        return coords

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            try:
                parts = line.split()
                if len(parts) < 3:
                    print(f"[Warning] Skipping invalid line: {line}")
                    continue

                name = parts[0].strip()
                lat = float(parts[1])
                lon = float(parts[2])
                coords[name] = (lat, lon)
            except ValueError:
                print(f"[Warning] Skipping invalid line: {line}")

    return coords


# =================== MQTT CALLBACKS ===================
def on_connect(client, userdata, flags, rc):
    print(f"[MQTT] Connected with result code {rc}")
    client.subscribe("chatpilot/rover/command")

def on_message(client, userdata, msg):
    global vehicle
    command_str = msg.payload.decode().strip().upper()
    print(f"[MQTT] Received command: {command_str}")

    if command_str == "FORWARD":
        print("[System] Moving forward...")
        # play_sound(forward_sound)
        move_relative(vehicle, 0.00001, 0)

    elif command_str == "LEFT":
        print("[System] Turning left...")
        # play_sound(left_sound)
        move_relative(vehicle, 0, -0.00001)

    elif command_str == "RIGHT":
        print("[System] Turning right...")
        # play_sound(right_sound)
        move_relative(vehicle, 0, 0.00001)

    elif command_str == "BACKWARD":
        print("[System] Moving backward...")
        move_relative(vehicle, -0.00001, 0)

    elif command_str == "STOP":
        print("[System] Stopping...")
        # play_sound(stop_sound)
        vehicle.mode = VehicleMode("HOLD")

    elif command_str.startswith("NAVIGATE:"):
        try:
            # Format: NAVIGATE:0,7
            parts = command_str.split(":")[1].split(",")
            if len(parts) != 2:
                print("[Error] Invalid NAVIGATE format. Use NAVIGATE:START,END")
                return

            start = parts[0].strip()
            end = parts[1].strip()

            if start not in location_map or end not in location_map:
                print(f"[Error] Unknown location(s): {start}, {end}")
                return

            start_coords = location_map[start]
            end_coords = location_map[end]
            print(f"[System] Navigating from {start} -> {end}")

            # For now, direct navigation (no A* path)
            goto_position(vehicle, LocationGlobalRelative(*end_coords, ALTITUDE))
            print(f"[System] Navigation complete: {start} -> {end}")

        except Exception as e:
            print(f"[Error] NAVIGATE command failed: {e}")


# =================== FUNCTIONS ===================
def move_relative(vehicle, dlat, dlon):
    """Move rover slightly in the given lat/lon direction"""
    if not vehicle:
        print("[Error] Vehicle not connected.")
        return
    current = vehicle.location.global_relative_frame
    target = LocationGlobalRelative(current.lat + dlat, current.lon + dlon, ALTITUDE)
    vehicle.simple_goto(target)
    print(f"[System] Moving to relative position {target.lat}, {target.lon}")

def get_haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates in meters"""
    R = 6371000  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = math.sin(d_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def goto_position(vehicle, target_location):
    """Go to a GPS location using DroneKit simple_goto"""
    print(f"[Navigation] Moving to {target_location.lat}, {target_location.lon}")
    vehicle.simple_goto(target_location)

    while True:
        current = vehicle.location.global_relative_frame
        distance = get_haversine_distance(current.lat, current.lon,
                                          target_location.lat, target_location.lon)
        print(f"[Navigation] Distance to target: {distance:.2f} meters")

        if distance <= WAYPOINT_REACHED_RADIUS:
            print("[Navigation] Target reached.")
            break
        time.sleep(1)


# =================== MAIN ===================
def main():
    global vehicle, location_map

    # Load coordinates
    print("[System] Reading coordinates from file...")
    location_map = read_coordinates_from_file(FILE_NAME)

    if not location_map:
        print("[Error] No valid coordinates found. Exiting...")
        return

    print("[System] Loaded named coordinates:")
    for name, (lat, lon) in location_map.items():
        print(f"  {name} -> Latitude: {lat:.7f}, Longitude: {lon:.7f}")

    # Initialize MQTT
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("13.232.191.178", 1883, 60)
    client.loop_start()

    print("[System] Connecting to Pixhawk...")
    vehicle = connect(PIXHAWK_PORT, baud=BAUDRATE, wait_ready=True)
    print("[System] Pixhawk connected successfully.")

    print("[System] Arming vehicle...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("[System] Waiting for arming...")
        time.sleep(1)
    print("[System] Vehicle armed and ready.")

    try:
        while True:
            time.sleep(1)  # Idle loop waiting for MQTT commands
    except KeyboardInterrupt:
        print("[System] Interrupted. Disarming...")
        vehicle.mode = VehicleMode("HOLD")
        vehicle.armed = False
    finally:
        client.loop_stop()
        client.disconnect()
        vehicle.close()
        print("[System] Shutdown complete.")


if __name__ == "__main__":
    main()
