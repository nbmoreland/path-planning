from pybricks.parameters import Port, Direction, Axis, Color
from pybricks.robotics import DriveBase
from pybricks.pupdevices import Motor
from pybricks.hubs import PrimeHub
import constants

# Initialize the hub and motors
hub = PrimeHub()
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, Direction.CLOCKWISE)

# Constants and grid setup
GRID_SIZE = 0.305  # in meters
INF = float('inf')


def initialize_grid(rows, cols):
    """Create initial grids for obstacles and Manhattan distances."""
    grid = [[INF] * cols for _ in range(rows)]
    obstacle_map = [[0] * cols for _ in range(rows)]
    return grid, obstacle_map


grid, obstacle_map = initialize_grid(constants.num_rows, constants.num_cols)

# Convert start and goal coordinates from constants to grid indices
start_coords = (round(
    constants.start[0] / GRID_SIZE) - 1, round(constants.start[1] / GRID_SIZE) - 1)
goal_coords = (round(constants.goal[0] / GRID_SIZE) - 1,
               round(constants.goal[1] / GRID_SIZE) - 1)
orientation = constants.orientation


def load_obstacles():
    """Load obstacles from constants.py and convert to grid coordinates."""
    for obstacle in constants.obstacles:
        x, y = (round(coord / GRID_SIZE) - 1 for coord in obstacle)
        obstacle_map[x][y] = 1
        grid[x][y] = INF


def load_manhattan(grid, start_x, start_y):
    """BFS to fill grid with Manhattan distances from goal."""
    flooded = [[False] * constants.num_cols for _ in range(constants.num_rows)]
    frontier = [(start_x, start_y)]
    grid[start_x][start_y] = 0
    distance = 1

    while frontier:
        new_frontier = []
        for x, y in frontier:
            if not flooded[x][y] and not obstacle_map[x][y]:
                flooded[x][y] = True
                for nx, ny in [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]:
                    if 0 <= nx < constants.num_rows and 0 <= ny < constants.num_cols and not flooded[nx][ny]:
                        grid[nx][ny] = min(grid[nx][ny], distance)
                        new_frontier.append((nx, ny))
        frontier = new_frontier
        distance += 1


def generate_path(orientation):
    """Generate path based on grid values and orientation."""
    x, y = start_coords
    path = ''
    while grid[x][y] != 0:
        x_next, y_next = x, y
        if orientation == 1:  # Facing +x
            x_next += 1
        elif orientation == -1:  # Facing -x
            x_next -= 1
        elif orientation == 1j:  # Facing +y
            y_next += 1
        elif orientation == -1j:  # Facing -y
            y_next -= 1

        if (0 <= x_next < constants.num_rows and 0 <= y_next < constants.num_cols and
                grid[x_next][y_next] < grid[x][y]):
            path += 'F'
            x, y = x_next, y_next
        else:
            path += 'L'
            orientation *= 1j  # Rotate 90 degrees left
    return path.replace('LLL', 'R')  # Simplify turns


def execute_path(path, orientation):
    """Execute path commands: F (move forward), L (turn left), R (turn right)."""
    drive_base = DriveBase(left_motor, right_motor,
                           wheel_diameter=56, axle_track=125)
    drive_base.use_gyro(True)

    x, y = start_coords
    print(f"{start_coords} => {goal_coords}")
    while path:
        print(f"Traversing Path: {path}")
        print(f"Location: ({x}, {y})")
        if path[0] == 'F':
            drive_base.straight(GRID_SIZE * 1000)
            path = path[1:]
        elif path[0] == 'L':
            drive_base.turn(-90)
            orientation *= 1j  # Rotate left
            path = path[1:]
        elif path[0] == 'R':
            drive_base.turn(90)
            orientation *= -1j  # Rotate right
            path = path[1:]


load_obstacles()
load_manhattan(grid, goal_coords[0], goal_coords[1])

path = generate_path(constants.orientation)
execute_path(path, constants.orientation)
