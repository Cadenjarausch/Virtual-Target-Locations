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

    def plot(self):
        plt.plot(self.x, self.y, 'bo')  # Plot the robot position as a blue dot

def simulate_until_boundary(vel, max_yaw_rate, dT, initial_yaw, initial_x, initial_y, environment):
    robot = Unicycle(x=initial_x, y=initial_y, theta=initial_yaw)  # Initialize with specified initial position and yaw

    # Lists to store trajectory
    x_traj, y_traj = [robot.x], [robot.y]

    while True:
        # Calculate possible endpoints for different yaw rates
        possible_endpoints = []
        for yaw in np.arange(-max_yaw_rate, max_yaw_rate, 0.1):
            temp_robot = Unicycle(robot.x, robot.y, robot.theta)  # Create a temporary robot instance
            temp_robot.move(vel, yaw, dT)  # Move the temporary robot
            possible_endpoints.append((temp_robot.x, temp_robot.y))

        # Check if any endpoint is outside the environment
        for endpoint in possible_endpoints:
            if not (environment[0] <= endpoint[0] <= environment[1] and environment[2] <= endpoint[1] <= environment[3]):
                return x_traj, y_traj

        # Move the robot based on the maximum yaw rate
        robot.move(vel, max_yaw_rate, dT)

        # Append new position to trajectory
        x_traj.append(robot.x)
        y_traj.append(robot.y)

# Example usage:
def plot_trajectory(x_traj, y_traj, environment):
    plt.figure()
    plt.plot(x_traj, y_traj, label='Unicycle trajectory', color='blue')
    plt.plot(environment[0], environment[2], 'rx', label='Environment boundary')  # Plot environment boundary
    plt.plot(environment[0], environment[3], 'rx')
    plt.plot(environment[1], environment[2], 'rx')
    plt.plot(environment[1], environment[3], 'rx')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Unicycle Trajectory until Boundary')
    plt.legend()
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

environment = (-50, 50, -50, 50)  # Define environment boundaries (x_min, x_max, y_min, y_max)
x_traj, y_traj = simulate_until_boundary(1.0, np.pi/4, 1.0, np.pi/6, 2.0, 2.0, environment)
plot_trajectory(x_traj, y_traj, environment)
