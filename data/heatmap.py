import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import KDTree

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


def plot_3d_scatter(x, y, z, densities):
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

  
    sc = ax.scatter(x, y, z, c=densities, cmap='coolwarm', marker='o',s=12)

    plt.colorbar(sc, label='visitation distribution')

    ax.scatter(start[0], start[1], start[2], c='red', marker='o', s=100, label="Start")
    ax.scatter(goal[0], goal[1], goal[2], c='green', marker='o', s=100, label="Goal")


    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_zlabel('Z Coordinate')
    ax.set_title('3D Scatter Plot Colored by Density')

    plt.show()

if __name__ == '__main__':
    file_path = 'predicted_paths.txt' 
    x, y, z = read_coordinates(file_path)
    densities = calculate_density(x, y, z, radius=0.3)

    start = [0.307, 0, 0.487]
    goal = [0.5, 0.5, 0.3]

    plot_3d_scatter(x, y, z, densities)
