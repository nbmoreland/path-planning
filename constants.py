# Define grid dimensions
num_rows = 15
num_cols = 10
tile_size_m = 0.305  # Each tile is 0.305 meters (1 foot)

# Define obstacles as grid coordinates and convert to meters
obstacle_coords = [
    (4, 1), (4, 2), (4, 3), (4, 4), (4, 5),
    (7, 4), (7, 5), (7, 6), (7, 7), (7, 8), (7, 9), (7, 10),
    (10, 3), (10, 4), (10, 5), (10, 6), (10, 7),
    (11, 3), (12, 3), (12, 4), (13, 3), (13, 4)
]

# Convert obstacle grid coordinates to meters
obstacles = [(x * tile_size_m, y * tile_size_m) for x, y in obstacle_coords]

# Print obstacle positions in meters
print("Obstacles:", obstacles)

# Define start and goal locations
start = (2 * tile_size_m, 2 * tile_size_m)
goal = (13 * tile_size_m, 7 * tile_size_m)

print("Start:", start)
print("Goal:", goal)

# Define initial orientation of the robot
# Orientation: 1 (facing positive x), 1j (positive y), -1 (negative x), -1j (negative y)
orientation = 1j

# Display robot orientation info
orientation_dict = {
    1: "positive x-axis",
    1j: "positive y-axis",
    -1: "negative x-axis",
    -1j: "negative y-axis"
}
print("Initial Orientation:", orientation_dict[orientation])
print("\n")
