import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

class RRRobot:
    def __init__(self, L1=40, L2=30):
        self.L1 = L1  # Length of first link (cm)
        self.L2 = L2  # Length of second link (cm)
        
    def forward_kinematics(self, theta1, theta2):
        """Calculate end-effector position given joint angles"""
        x = self.L1 * np.cos(theta1) + self.L2 * np.cos(theta1 + theta2)
        y = self.L1 * np.sin(theta1) + self.L2 * np.sin(theta1 + theta2)
        return x, y

    def inverse_kinematics(self, x, y):
        """Calculate joint angles given end-effector position"""
        r = np.sqrt(x**2 + y**2)
        if r > (self.L1 + self.L2) or r < abs(self.L1 - self.L2):
            return None  # Point is unreachable
        
        # Cosine law for theta2
        cos_theta2 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))
        
        # Calculate theta1
        theta1 = np.arctan2(y, x) - np.arctan2(self.L2 * np.sin(theta2),
                                              self.L1 + self.L2 * np.cos(theta2))
        
        return theta1, theta2

def plot_workspace_and_trajectory():
    # Create figure
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111)
    
    # Initialize robot
    robot = RRRobot()
    
    # Plot workspace boundary
    theta1_range = np.linspace(-np.pi, np.pi, 100)
    theta2_range = np.linspace(-np.pi, np.pi, 100)
    x_points = []
    y_points = []
    
    for theta1 in theta1_range:
        for theta2 in theta2_range:
            x, y = robot.forward_kinematics(theta1, theta2)
            x_points.append(x)
            y_points.append(y)
    
    # Plot workspace points
    ax.scatter(x_points, y_points, c='lightgray', alpha=0.1, s=1)
    
    # Plot target area
    target_x = [20, 50, 50, 20, 20]
    target_y = [-15, -15, 15, 15, -15]
    ax.plot(target_x, target_y, 'r--', label='Target Area')
    
    # Generate trajectory (diagonal line from bottom-left to top-right)
    start_point = (20, -15)  # Bottom-left corner
    end_point = (50, 15)    # Top-right corner
    
    # Generate trajectory points
    num_points = 50
    trajectory_x = np.linspace(start_point[0], end_point[0], num_points)
    trajectory_y = np.linspace(start_point[1], end_point[1], num_points)
    
    # Plot trajectory
    ax.plot(trajectory_x, trajectory_y, 'g-', label='Trajectory')
    
    # Plot robot links for initial position
    theta1, theta2 = robot.inverse_kinematics(start_point[0], start_point[1])
    if theta1 is not None and theta2 is not None:
        # Plot first link
        x1 = robot.L1 * np.cos(theta1)
        y1 = robot.L1 * np.sin(theta1)
        ax.plot([0, x1], [0, y1], 'b-', linewidth=2, label='Link 1')
        
        # Plot second link
        x2, y2 = robot.forward_kinematics(theta1, theta2)
        ax.plot([x1, x2], [y1, y2], 'g-', linewidth=2, label='Link 2')
    
    # Set plot properties
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_title('RR Robot Workspace and Trajectory')
    ax.grid(True)
    ax.axis('equal')
    ax.legend()
    
    # Show the plot
    plt.show()

# Plot the workspace and trajectory
plot_workspace_and_trajectory()

# Function to animate robot movement along trajectory
def animate_trajectory():
    robot = RRRobot()
    
    # Create figure for animation
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(-60, 60)
    ax.set_ylim(-60, 60)
    
    # Generate trajectory points
    start_point = (20, -15)
    end_point = (50, 15)
    num_points = 100
    trajectory_x = np.linspace(start_point[0], end_point[0], num_points)
    trajectory_y = np.linspace(start_point[1], end_point[1], num_points)
    
    # Animation update function
    line1, = ax.plot([], [], 'b-', linewidth=2)  # First link
    line2, = ax.plot([], [], 'g-', linewidth=2)  # Second link
    
    def update(frame):
        # Calculate inverse kinematics for current position
        theta1, theta2 = robot.inverse_kinematics(trajectory_x[frame], trajectory_y[frame])
        
        if theta1 is not None and theta2 is not None:
            # Calculate positions for links
            x1 = robot.L1 * np.cos(theta1)
            y1 = robot.L1 * np.sin(theta1)
            x2, y2 = robot.forward_kinematics(theta1, theta2)
            
            # Update link positions
            line1.set_data([0, x1], [0, y1])
            line2.set_data([x1, x2], [y1, y2])
        
        return line1, line2
    
    # Create animation
    ani = animation.FuncAnimation(fig, update, frames=num_points,
                                interval=50, blit=True)
    
    # Plot target area
    target_x = [20, 50, 50, 20, 20]
    target_y = [-15, -15, 15, 15, -15]
    ax.plot(target_x, target_y, 'r--', label='Target Area')
    
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_title('RR Robot Trajectory Animation')
    ax.grid(True)
    ax.axis('equal')
    ax.legend()
    
    plt.show()

# Run the animation
animate_trajectory()