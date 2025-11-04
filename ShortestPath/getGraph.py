import math

# -------------------- HAVERSINE FUNCTION --------------------
def haversine_distance(coord1, coord2):
    """Calculate great-circle distance (in meters) between two coordinates."""
    R = 6371000  # Earth radius in meters

    lat1, lon1, _ = coord1
    lat2, lon2, _ = coord2

    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)

    a = math.sin(d_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c


# -------------------- READ INPUT FILES --------------------
def read_coordinates(filename):
    """Read coordinates from cords.txt and return a dict."""
    coords = {}
    with open(filename, 'r') as f:
        for line in f:
            if line.strip():
                parts = line.strip().split()
                node = int(parts[0])
                lat, lon, alt = map(float, parts[1:])
                coords[node] = (lat, lon, alt)
    return coords


def read_connections(filename):
    """Read adjacency list from list.txt and return a dict."""
    connections = {}
    with open(filename, 'r') as f:
        for line in f:
            if ':' not in line:
                continue
            node, neighbors = line.strip().split(':')
            node = int(node.strip())
            neighbors = [int(n.strip()) for n in neighbors.split(',') if n.strip()]
            connections[node] = neighbors
    return connections


# -------------------- BUILD GRAPH STRUCTURES --------------------
def create_adjacency_list(coords, connections):
    """Return weighted adjacency list using Haversine distance."""
    adj_list = {}
    for node, neighbors in connections.items():
        adj_list[node] = {}
        for neighbor in neighbors:
            if neighbor not in coords:
                print(f"⚠️ Warning: Neighbor {neighbor} not found in coordinates. Skipping.")
                continue
            dist = haversine_distance(coords[node], coords[neighbor])
            adj_list[node][neighbor] = round(dist, 2)
    return adj_list



# -------------------- SAVE OUTPUT FILES --------------------
def save_adjacency_list(filename, adj_list):
    with open(filename, 'w') as f:
        for node, neighbors in adj_list.items():
            line = f"{node}: " + ", ".join(f"{nbr} ({dist}m)" for nbr, dist in neighbors.items())
            f.write(line + "\n")


def save_adjacency_matrix(filename, matrix):
    with open(filename, 'w') as f:
        for row in matrix:
            f.write(" ".join(f"{val:.2f}" for val in row) + "\n")


# -------------------- MAIN --------------------
if __name__ == "__main__":
    coords = read_coordinates("cords.txt")
    connections = read_connections("list.txt")

    adj_list = create_adjacency_list(coords, connections)


    save_adjacency_list("weighted_adj_list.txt", adj_list)

    print("✅ Weighted adjacency list saved to 'weighted_adj_list.txt'")

