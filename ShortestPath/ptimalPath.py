from geopy.distance import geodesic
import itertools

# --- Define GPS coordinates ---
A = (10.12345, 76.45678)
B = (10.12345, 76.45800)
C = (10.12480, 76.45800)
D = (10.12480, 76.45678)

# Rover current position
rover = (10.12400, 76.45720)

# Store corners in dictionary
corners = {'A': A, 'B': B, 'C': C, 'D': D}

# --- Step 1: Find nearest corner ---
nearest_corner = min(corners, key=lambda k: geodesic(rover, corners[k]).meters)
print(f"Nearest corner to rover: {nearest_corner}")

# --- Step 2: Generate possible rectangular paths ---
# Clockwise and counterclockwise around the rectangle
paths = {
    "CW": ['A', 'B', 'C', 'D'],
    "CCW": ['A', 'D', 'C', 'B']
}

# Rotate path so it starts from nearest corner
def rotate_to_start(path, start):
    idx = path.index(start)
    return path[idx:] + path[:idx]

# Compute distance for both directions
best_path = None
min_distance = float('inf')

for name, path in paths.items():
    path = rotate_to_start(path, nearest_corner)
    # Add back to start to complete loop if needed
    total = 0
    for i in range(len(path)):
        p1 = corners[path[i]]
        p2 = corners[path[(i + 1) % len(path)]]
        total += geodesic(p1, p2).meters
    if total < min_distance:
        min_distance = total
        best_path = path

# --- Step 3: Display results ---
print(f"Optimal Path: {' → '.join(best_path)}")
print(f"Total Distance: {min_distance:.2f} meters")