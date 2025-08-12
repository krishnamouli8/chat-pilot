from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

coordinates = []
def read_coordinates_from_file(filename):
    

    try:
        with open(filename, 'r') as file:
            print(file)
            for line in file:
                line = line.replace(" ", "")
                parts = line.strip().split(',')
                if True:
                    try:
                        lat = float(parts[0])
                        lon = float(parts[1])
                        coordinates.append(LocationGlobalRelative(lat, lon, 0.0))
                    except ValueError:
                        print(f"Skipping invalid line: {line.strip()}")
    except FileNotFoundError:
        print(f"File not found: {filename}")

def goto_waypoint(location):
    print(f"Flying to: Lat {location.lat}, Lon {location.lon}")
    vehicle.simple_goto(location)
    time.sleep(20)  # Adjust this depending on distance



# Connect to the Vehicle
print("Connecting to vehicle...")
vehicle = connect('127.0.0.1:5761', wait_ready=True)
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

read_coordinates_from_file("cords.txt")
for c in coordinates:
    goto_waypoint(c)