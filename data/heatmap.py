import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import KDTree
import yaml

def read_coordinates(file_path):
    coordinates = np.loadtxt(file_path)
    return coordinates[:, 0], coordinates[:, 1], coordinates[:, 2]  

def calculate_density(x, y, z, radius=1.0):
    points = np.vstack((x, y, z)).T
    tree = KDTree(points)
    densities = []

    for point in points:
        neighbors = tree.query_ball_point(point, radius)
        densities.append(len(neighbors))

    return np.array(densities)

def read_real_path(file_path):
    real_path = np.loadtxt(file_path)
    return real_path[:, 0], real_path[:, 1], real_path[:, 2]

def read_obstacles_from_yaml(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)

    obstacles = data['obstacles']
    obstacle_positions = []
    obstacle_radii = []

    for obstacle in obstacles:
        position = obstacle['position']
        radius = obstacle['description']['dimensions'][0]
        obstacle_positions.append(position)
        obstacle_radii.append(radius)

    return np.array(obstacle_positions), np.array(obstacle_radii)

def plot_3d_scatter(x, y, z, densities, start, goal, obstacles, obstacle_radii, real_path_x, real_path_y, real_path_z):
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_ylim([0, 1])
    ax.set_zlim([0, 1])
    ax.set_xlim([0, 1])
    sc = ax.scatter(x, y, z, c=densities, cmap='coolwarm', marker='o', s=2, alpha=0.1)

    plt.colorbar(sc, label='Visitation Distribution')

    ax.scatter(start[0], start[1], start[2], c='red', marker='o', s=100, label="Start")
    ax.scatter(goal[0], goal[1], goal[2], c='green', marker='o', s=100, label="Goal")
    
    for i, (position, radius) in enumerate(zip(obstacles, obstacle_radii)):
        u, v = np.mgrid[0:2*np.pi:8j, 0:np.pi:8j]
        x_sphere = position[0] + radius * np.cos(u) * np.sin(v)
        y_sphere = position[1] + radius * np.sin(u) * np.sin(v)
        z_sphere = position[2] + radius * np.cos(v)
        
        ax.plot_surface(x_sphere, y_sphere, z_sphere, color='white', edgecolor='purple', alpha=0.1, linewidth=0.15)

    ax.plot(real_path_x, real_path_y, real_path_z, color='black', linewidth=0.5, label="Real Trajectory")
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_zlabel('Z Coordinate')
    ax.set_title('3D Scatter Plot Colored by Density')
    
    ax.view_init(elev=21, azim=118)  

    ax.legend()

    plt.show()

if __name__ == '__main__':
    real_path_file = 'real_path.txt' 
    file_path = 'predicted_paths.txt' 

    ## Need to change to fit the scene'''
    yaml_file_path = '/home/geriatronics/FLIQC_example_workspace_ros/src/robot_env_publisher/scene/figureA11.yaml'

    x, y, z = read_coordinates(file_path)
    densities = calculate_density(x, y, z, radius=0.3)

    ## Need to change to fit the scene
    start = [0.307, 0, 0.487]
    goal = [0.5, 0.5, 0.5]

    obstacles, obstacle_radii = read_obstacles_from_yaml(yaml_file_path)
    real_path_x, real_path_y, real_path_z = read_real_path(real_path_file)

    plot_3d_scatter(x, y, z, densities, start, goal, obstacles, obstacle_radii, real_path_x, real_path_y, real_path_z)
