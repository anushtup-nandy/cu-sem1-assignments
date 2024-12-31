import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

class RRRobot3D:
    def __init__(self, L1=40, L2=30):
        self.L1 = L1  # Length of first link (cm)
        self.L2 = L2  # Length of second link (cm)
        
    def forward_kinematics(self, theta1, theta2):
        """Calculate end-effector position given joint angles
        theta1: rotation about z-axis
        theta2: rotation about x-axis (after theta1 rotation)
        """
        # First link position (after theta1 rotation about z)
        x1 = self.L1 * np.cos(theta1)
        y1 = self.L1 * np.sin(theta1)
        z1 = 0
        
        # Second link position (after theta2 rotation about local x)
        # We need to rotate the second link about the local x-axis at the end of the first link
        x2 = x1 + self.L2 * np.cos(theta1) * np.cos(theta2)
        y2 = y1 + self.L2 * np.sin(theta1) * np.cos(theta2)
        z2 = self.L2 * np.sin(theta2)
        
        return x2, y2, z2

def plot_workspace_and_trajectory():
    # Create 3D figure
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Initialize robot
    robot = RRRobot3D()
    
    # Plot workspace boundary
    theta1_range = np.linspace(-np.pi, np.pi, 30)
    theta2_range = np.linspace(-np.pi/2, np.pi/2, 30)  # Limit theta2 to avoid self-collision
    
    # Create mesh grid for surface plot
    THETA1, THETA2 = np.meshgrid(theta1_range, theta2_range)
    X = np.zeros_like(THETA1)
    Y = np.zeros_like(THETA1)
    Z = np.zeros_like(THETA1)
    
    # Calculate workspace points
    for i in range(THETA1.shape[0]):
        for j in range(THETA1.shape[1]):
            x, y, z = robot.forward_kinematics(THETA1[i,j], THETA2[i,j])
            X[i,j] = x
            Y[i,j] = y
            Z[i,j] = z
    
    # Plot workspace surface
    ax.plot_surface(X, Y, Z, alpha=0.2, cmap='viridis')
    
    # Plot target volume (rectangular prism)
    x_min, x_max = 20, 50
    y_min, y_max = -15, 15
    z_min, z_max = 0, 0.001
    
    # Create target volume vertices
    vertices = np.array([
        [x_min, y_min, z_min], [x_max, y_min, z_min], [x_max, y_max, z_min], [x_min, y_max, z_min],
        [x_min, y_min, z_max], [x_max, y_min, z_max], [x_max, y_max, z_max], [x_min, y_max, z_max]
    ])
    
    # Define edges of the target volume
    edges = [
        [0,1], [1,2], [2,3], [3,0],  # Bottom face
        [4,5], [5,6], [6,7], [7,4],  # Top face
        [0,4], [1,5], [2,6], [3,7]   # Vertical edges
    ]
    
    # Plot target volume edges
    for edge in edges:
        ax.plot3D(vertices[edge, 0], vertices[edge, 1], vertices[edge, 2], 'r--')
    
    # Generate trajectory (diagonal line from bottom corner to opposite top corner)
    start_point = vertices[0]  # Bottom front left corner
    end_point = vertices[6]    # Top back right corner
    
    # Generate trajectory points
    num_points = 50
    trajectory_x = np.linspace(start_point[0], end_point[0], num_points)
    trajectory_y = np.linspace(start_point[1], end_point[1], num_points)
    trajectory_z = np.linspace(start_point[2], end_point[2], num_points)
    
    # Plot trajectory
    ax.plot3D(trajectory_x, trajectory_y, trajectory_z, 'g-', linewidth=2, label='Trajectory')
    
    # Set plot properties
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_zlabel('Z (cm)')
    ax.set_title('3D RR Robot Workspace and Trajectory')
    
    # Show the plot
    plt.show()

def animate_robot_movement():
    # Create 3D figure
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    robot = RRRobot3D()
    
    # Generate trajectory points
    start_point = np.array([20, -15, 0])  # Bottom front left corner
    end_point = np.array([50, 15, 0.001])    # Top back right corner
    num_frames = 100
    
    # Create trajectory
    t = np.linspace(0, 1, num_frames)
    # Use smooth interpolation
    t = 0.5 * (1 - np.cos(t * np.pi))
    trajectory = np.array([(1-t_)*start_point + t_*end_point for t_ in t])

    # Initialize lines for robot links
    line1, = ax.plot([], [], [], 'b-', linewidth=2, label='Link 1')
    line2, = ax.plot([], [], [], 'g-', linewidth=2, label='Link 2')
    
    # Plot target volume (as reference)
    x_min, x_max = 20, 50
    y_min, y_max = -15, 15
    z_min, z_max = 0, 0.001
    vertices = np.array([
        [x_min, y_min, z_min], [x_max, y_min, z_min], [x_max, y_max, z_min], [x_min, y_max, z_min],
        [x_min, y_min, z_max], [x_max, y_min, z_max], [x_max, y_max, z_max], [x_min, y_max, z_max]
    ])
    edges = [
        [0,1], [1,2], [2,3], [3,0],
        [4,5], [5,6], [6,7], [7,4],
        [0,4], [1,5], [2,6], [3,7]
    ]
    for edge in edges:
        ax.plot3D(vertices[edge, 0], vertices[edge, 1], vertices[edge, 2], 'r--')

    def update(frame):
        # Clear previous frame
        ax.cla()
        
        # Plot target volume edges
        for edge in edges:
            ax.plot3D(vertices[edge, 0], vertices[edge, 1], vertices[edge, 2], 'r--')
        
        # Get current target point
        target = trajectory[frame]
        
        # Calculate inverse kinematics
        x, y, z = target[0], target[1], target[2]
        
        # Calculate distance from base to target in XY plane
        r_xy = np.sqrt(x**2 + y**2)
        
        # Calculate theta1 (base rotation)
        theta1 = np.arctan2(y, x)
        
        # Calculate angles for elbow-down configuration
        r = np.sqrt(r_xy**2 + z**2)
        
        # Law of cosines for theta2
        cos_alpha = (robot.L1**2 + r**2 - robot.L2**2) / (2 * robot.L1 * r)
        cos_alpha = np.clip(cos_alpha, -1, 1)
        alpha = np.arccos(cos_alpha)
        
        # Calculate elevation angle from XY plane to target
        beta = np.arctan2(z, r_xy)
        
        # Final theta2 (shoulder angle)
        theta2 = beta - alpha  # Use minus for elbow-down configuration
        
        # Calculate joint positions
        # First joint (shoulder)
        x1 = robot.L1 * np.cos(theta1) * np.cos(theta2)
        y1 = robot.L1 * np.sin(theta1) * np.cos(theta2)
        z1 = robot.L1 * np.sin(theta2)
        
        # End effector position
        x2 = x  # Should match target x
        y2 = y  # Should match target y
        z2 = z  # Should match target z
        
        # Plot robot links
        ax.plot3D([0, x1], [0, y1], [0, z1], 'b-', linewidth=2)
        ax.plot3D([x1, x2], [y1, y2], [z1, z2], 'g-', linewidth=2)
        
        # Plot target point
        ax.scatter(target[0], target[1], target[2], color='red', marker='o')
        
        # Set axes properties
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')
        ax.set_zlabel('Z (cm)')
        ax.set_title(f'Frame {frame}/{num_frames-1}')
        ax.set_xlim([-60, 60])
        ax.set_ylim([-60, 60])
        ax.set_zlim([-20, 40])

    ani = animation.FuncAnimation(fig, update, frames=num_frames,
                                 interval=50, repeat=True)
    plt.show()

# Run the visualizations
print("Showing workspace and trajectory plot...")
plot_workspace_and_trajectory()

print("\nShowing robot animation...")
animate_robot_movement()