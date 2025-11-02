from dronekit import connect, VehicleMode, LocationGlobalRelative
from rplidar import RPLidar, RPLidarException
import math, time, serial, json, threading, sys
import numpy as np
import paho.mqtt.client as mqtt
import pyttsx3
from AStarSearch import a_star

# =================== CONFIG ===================
PIXHAWK_PORT = '/dev/ttyACM0'
DDSM_PORT = '/dev/ttyACM1'
LIDAR_PORT = '/dev/ttyUSB0'
SERIAL_BAUDRATE = 115200
BAUDRATE = 57600

ALTITUDE = 0.0
WAYPOINT_REACHED_RADIUS = 1
FORWARD_SPEED = 40
TURN_SPEED = 40
FORWARD_AVOID_TIME = 7  # seconds to go straight during avoidance

STOP_DISTANCE = 1000  # mm threshold for obstacle detection
CLEAR_DISTANCE = 1100 # mm to confirm clear
FRONT_SECTOR = 30
SIDE_SECTOR = 45
LOOP_DELAY = 0.1

vehicle = None
engine = None
path = []
path_total_cost = 0.0
latest_servo1_value = None
latest_servo3_value = None

obstacle_detected = False
current_command = None  # store last movement command

# =================== SERIAL SETUP ===================
try:
    ddsm_ser = serial.Serial(DDSM_PORT, baudrate=SERIAL_BAUDRATE)
    ddsm_ser.setRTS(False)
    ddsm_ser.setDTR(False)
    print("[System] ✅ DDSM Connected")
except Exception as e:
    print(f"[Error] Cannot open DDSM port: {e}")
    sys.exit(1)

# =================== MOTOR CONTROL ===================
def motor_control(left, right):
    cmd_right = {"T": 10010, "id": 2, "cmd": -right, "act": 3}
    cmd_left = {"T": 10010, "id": 1, "cmd": left, "act": 3}
    ddsm_ser.write((json.dumps(cmd_right) + '\n').encode())
    time.sleep(0.01)
    ddsm_ser.write((json.dumps(cmd_left) + '\n').encode())

# =================== MOVEMENT HELPERS ===================
def stop_rover():
    motor_control(0, 0)
    print("[Action] 🛑 Rover stopped.")

def move_forward(speed=FORWARD_SPEED, duration=None):
    motor_control(speed, speed)
    if duration:
        time.sleep(duration)
        stop_rover()

def turn_left(duration=1.5):
    print("[Action] ↩️ Turning LEFT...")
    motor_control(-TURN_SPEED, TURN_SPEED)
    time.sleep(duration)
    stop_rover()

def turn_right(duration=1.5):
    print("[Action] ↪️ Turning RIGHT...")
    motor_control(TURN_SPEED, -TURN_SPEED)
    time.sleep(duration)
    stop_rover()

# =================== MQTT CALLBACKS ===================
def on_connect(client, userdata, flags, rc):
    print(f"MQTT_CLIENT::Connected with result code {rc}")
    client.subscribe("chatpilot/rover/command")

def on_message(client, userdata, msg):
    global current_command, obstacle_detected

    command = msg.payload.decode().strip().upper()
    print(f"MQTT_CLIENT::Received command: {command}")

    # Save current movement command for resuming later
    if command in ["FORWARD", "BACKWARD", "LEFT", "RIGHT"]:
        current_command = command

    if obstacle_detected and command not in ["STOP"]:
        print("[System] 🚫 Obstacle detected — ignoring command.")
        return

    if command == "FORWARD":
        motor_control(FORWARD_SPEED, FORWARD_SPEED)
    elif command == "BACKWARD":
        motor_control(-FORWARD_SPEED, -FORWARD_SPEED)
    elif command == "LEFT":
        turn_left()
    elif command == "RIGHT":
        turn_right()
    elif command == "STOP":
        stop_rover()

# =================== LIDAR LOGIC ===================
def lidar_thread():
    global obstacle_detected, current_command

    try:
        lidar = RPLidar(LIDAR_PORT, timeout=3)
    except Exception as e:
        print(f"❌ Cannot open LiDAR on {LIDAR_PORT}: {e}")
        return

    print("[LIDAR] ✅ Connected. Starting motor...")

    try:
        lidar.stop()
        lidar.stop_motor()
        lidar.start_motor()
        time.sleep(2)
        iterator = lidar.iter_measurments(max_buf_meas=500)
        print("[LIDAR] 🔄 Scanning...")

        while True:
            readings = []
            for _ in range(80):
                try:
                    _, quality, angle, distance = next(iterator)
                    if distance > 0:
                        readings.append((angle, distance))
                except RPLidarException:
                    continue

            if not readings:
                continue

            angles = np.array([a for a, _ in readings])
            distances = np.array([d for _, d in readings])

            def sector_min(center, half_angle):
                mask = ((angles >= (center - half_angle)) & (angles <= (center + half_angle)))
                vals = distances[mask]
                return np.min(vals) if len(vals) else np.inf

            front = sector_min(0, FRONT_SECTOR)
            left = sector_min(45, SIDE_SECTOR)
            right = sector_min(315, SIDE_SECTOR)

            if front < STOP_DISTANCE:
                if not obstacle_detected:
                    obstacle_detected = True
                    print(f"[LIDAR] 🚨 Obstacle detected at {front:.0f} mm — stopping rover!")
                    stop_rover()
                    engine.say("Obstacle detected")
                    engine.runAndWait()

                    # Decision logic
                    if left > CLEAR_DISTANCE:
                        print("[LIDAR] ⬅️ Left side clear — performing left bypass.")
                        turn_left()
                        move_forward(FORWARD_SPEED, FORWARD_AVOID_TIME)
                        turn_right()
                    elif right > CLEAR_DISTANCE:
                        print("[LIDAR] ➡️ Right side clear — performing right bypass.")
                        turn_right()
                        move_forward(FORWARD_SPEED, FORWARD_AVOID_TIME)
                        turn_left()
                    else:
                        print("[LIDAR] ⛔ Both sides blocked — waiting.")
                        stop_rover()
                        while True:
                            # keep checking until clear
                            _, _, _, distance = next(iterator)
                            if distance > CLEAR_DISTANCE:
                                break
                            time.sleep(0.2)

                    print("[LIDAR] ✅ Path clear — resuming previous motion.")
                    obstacle_detected = False

                    # Resume previous command if any
                    if current_command == "FORWARD":
                        motor_control(FORWARD_SPEED, FORWARD_SPEED)
                    elif current_command == "BACKWARD":
                        motor_control(-FORWARD_SPEED, -FORWARD_SPEED)
                    elif current_command == "LEFT":
                        turn_left()
                    elif current_command == "RIGHT":
                        turn_right()

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\n[LIDAR] Stopping...")

    finally:
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except:
            pass
        print("[LIDAR] ✅ LiDAR disconnected")

# =================== MAIN ===================
def main():
    global engine

    # MQTT setup
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("13.232.191.178", 1883, 60)
    client.loop_start()

    engine = pyttsx3.init()
    print("[System] Connecting to Pixhawk...")
    vehicle = connect(PIXHAWK_PORT, baud=BAUDRATE, wait_ready=False)
    print("[System] ✅ Pixhawk connected.")

    # Start LIDAR obstacle avoidance in background
    threading.Thread(target=lidar_thread, daemon=True).start()

    # Arm Pixhawk (optional if needed)
    print("[System] Arming vehicle...")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    print(f"[System] Vehicle Mode → {vehicle.mode.name}")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_rover()
        print("[System] Shutdown requested.")
    finally:
        try:
            vehicle.armed = False
            vehicle.close()
        except:
            pass
        client.loop_stop()
        client.disconnect()
        print("[System] Shutdown complete.")

if __name__ == "__main__":
    main()
