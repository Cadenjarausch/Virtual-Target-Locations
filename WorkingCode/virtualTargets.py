import numpy as np
import matplotlib.pyplot as plt

class Unicycle:
    def __init__(self, x = 0, y = 0, theta = 0):
        self.x = x
        self.y = y
        self.theta = theta

    def move(self, vel, yaw, dT):
        self.x += vel * np.cos(self.theta + yaw) * dT
        self.y += vel * np.sin(self.theta + yaw) * dT
        self.theta += yaw

    def plot(self):
        plt.plot(self.x, self.y, 'bo')  # Plot the robot position as a blue dot

def simulate(vel, max_yaw_rate, dT, initial_yaw, initial_x, initial_y):
    robot = Unicycle(x = initial_x, y = initial_y, theta = initial_yaw)  # Initialize with specified initial position and yaw

    # Lists to store trajectory
    x_traj, y_traj = [robot.x], [robot.y]

    # Calculate possible endpoints for different yaw rates
    possible_endpoints = []
    for yaw in np.arange(-max_yaw_rate, max_yaw_rate, 0.1):
        temp_robot = Unicycle(robot.x, robot.y, robot.theta)  # Create a temporary robot instance
        temp_robot.move(vel, yaw, dT)  # Move the temporary robot
        possible_endpoints.append((temp_robot.x, temp_robot.y))

    # Find the highest and lowest points among the possible endpoints
    max_point = max(possible_endpoints, key = lambda item: item[1])
    min_point = min(possible_endpoints, key = lambda item: item[1])

    # Plot Unicycle
    plt.figure()
    robot.plot()

    # Plot trajectory lines from the unicycle's starting point to the highest and lowest points (red and dashed)
    plt.plot([robot.x, max_point[0]], [robot.y, max_point[1]], 'r--', alpha = 0.5, label='Trajectory Bounds')  # Trajectory line to highest point
    plt.plot([robot.x, min_point[0]], [robot.y, min_point[1]], 'r--', alpha = 0.5)  # Trajectory line to lowest point

    # Plot possible endpoints (solid green dots connected by a green line)
    x_endpoints, y_endpoints = zip(*possible_endpoints)
    plt.plot(x_endpoints, y_endpoints, 'g-', alpha = 0.5)  # Green line connecting endpoints
    plt.plot(x_endpoints, y_endpoints, 'go', alpha = 0.7, label='Virtual Targets')  # Solid green dots for endpoints

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Unicycle Model Simulation')
    plt.legend()
    plt.axis('equal')
    plt.grid()
    plt.show()

# Example usage:
simulate(1.0, np.pi/4, 1.0, np.pi/6, 2.0, 2.0)  # Velocity = 1.0, Max Yaw Rate = pi/4, dT = 1.0, Initial Yaw = pi/6, Initial X = 2.0, Initial Y = 2.0
