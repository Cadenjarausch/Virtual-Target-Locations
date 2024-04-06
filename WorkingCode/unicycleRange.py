import numpy as np
import matplotlib.pyplot as plt

class Unicycle:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

    def move(self, vel, yaw, dT):
        self.x += vel * np.cos(self.theta + yaw) * dT
        self.y += vel * np.sin(self.theta + yaw) * dT
        self.theta += yaw

def simulate_until_boundary(vel, max_yaw_rate, dT, initial_yaw, initial_x, initial_y, environment):
    robot = Unicycle(x=initial_x, y=initial_y, theta=initial_yaw)  # Initialize with specified initial position and yaw

    # Lists to store trajectories
    x_traj_right, y_traj_right = [], []
    x_traj_left, y_traj_left = [], []

    # For maximizing yaw to the right
    while True:
        # Calculate next position
        next_x = robot.x + vel * np.cos(robot.theta + max_yaw_rate) * dT
        next_y = robot.y + vel * np.sin(robot.theta + max_yaw_rate) * dT

        # Check if the next position falls outside the environment
        if not (environment[0] <= next_x <= environment[1] and environment[2] <= next_y <= environment[3]):
            break

        # Move the robot based on the maximum yaw rate
        robot.move(vel, max_yaw_rate, dT)

        # Append new position to trajectory for maximizing yaw to the right
        x_traj_right.append(robot.x)
        y_traj_right.append(robot.y)

    # Reset robot's position
    robot = Unicycle(x=initial_x, y=initial_y, theta=initial_yaw)

    # For maximizing yaw to the left
    while True:
        # Calculate next position
        next_x = robot.x + vel * np.cos(robot.theta - max_yaw_rate) * dT
        next_y = robot.y + vel * np.sin(robot.theta - max_yaw_rate) * dT

        # Check if the next position falls outside the environment
        if not (environment[0] <= next_x <= environment[1] and environment[2] <= next_y <= environment[3]):
            break

        # Move the robot based on the maximum yaw rate
        robot.move(vel, -max_yaw_rate, dT)

        # Append new position to trajectory for maximizing yaw to the left
        x_traj_left.append(robot.x)
        y_traj_left.append(robot.y)
        
    return x_traj_right, y_traj_right, x_traj_left, y_traj_left

def generate_targets(num_targets, environment):
    # Generate random targets within the environment boundaries
    targets_x = np.random.uniform(environment[0], environment[1], num_targets)
    targets_y = np.random.uniform(environment[2], environment[3], num_targets)
    return targets_x, targets_y

def can_reach_targets(robot_x_traj, robot_y_traj, targets_x, targets_y, vel, dT):
    # Initialize binary vector
    reachable = np.zeros(len(targets_x), dtype=int)
    
    # Loop through targets
    for i in range(len(targets_x)):
        # Find the index of the closest x value in the trajectory to the target's x-coordinate
        closest_index = np.argmin(np.abs(np.array(robot_x_traj) - targets_x[i]))
        
        # Check if the target's y-coordinate falls between the y-values of the trajectory at the corresponding x-coordinate
        if (min(robot_y_traj) <= abs(targets_y[i]) <= robot_y_traj[closest_index]):
            reachable[i] = 1
    
    return reachable

def plot_trajectory(x_traj_right, y_traj_right, x_traj_left, y_traj_left, environment, targets_x, targets_y, reachable_targets):
    plt.figure()
    plt.plot(x_traj_right[0], y_traj_right[0], "o", label='Starting point')  # Starting point
    plt.plot(x_traj_right, y_traj_right, color='blue')
    plt.plot(x_traj_left, y_traj_left, color='blue')
    
    # Plot environment boundary
    plt.plot([environment[0], environment[1], environment[1], environment[0], environment[0]], [environment[2], environment[2], environment[3], environment[3], environment[2]], 'rx--', label='Environment boundary')

    # Plot reachable targets
    for i in range(len(targets_x)):
        if reachable_targets[i] == 1:
            plt.plot(targets_x[i], targets_y[i], 'go')
        else:
            plt.plot(targets_x[i], targets_y[i], 'ko')
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Unicycle Trajectories and Reachable Targets')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    plt.grid(True)
    plt.show()

# Define environment boundaries (x_min, x_max, y_min, y_max)
environment = (0, 100, -50, 50)

# Simulation parameters
vel = 20.0
max_yaw_rate = np.pi / 100
dT = 0.1

# Generate targets
num_targets = 10
targets_x, targets_y = generate_targets(num_targets, environment)

# Simulate trajectories
x_traj_right, y_traj_right, x_traj_left, y_traj_left = simulate_until_boundary(vel, max_yaw_rate, dT, 0.0, 0.0, 0.0, environment)

# Check reachable targets
reachable_targets = can_reach_targets(x_traj_right, y_traj_right, targets_x, targets_y, vel, dT)
print(reachable_targets)

# Plot trajectories and reachable targets
plot_trajectory(x_traj_right, y_traj_right, x_traj_left, y_traj_left, environment, targets_x, targets_y, reachable_targets)
