from math import cos, sin
import matplotlib.pyplot as plt
from typing import Sequence

from environment import Environment
from utils import Bounds, Pose, Vector

class Visualizer:
    """
    A class to visualize results from simulations
    """

    def __init__(self) -> None:
        """
        Create the visualizer
        """

        # Set up the axes to plot on
        self.fig, self.ax = plt.subplots()

    def show(self):
        """
        Show the plot
        """
        plt.show()

    def plot_environment(self, environment: Environment):
        """
        Display an environment on the plot
        
        Params:
            environment: The environment to plot
        """
        # Plot each obstacle
        for obstacle in environment.obstacles:
            self.plot_bounds(obstacle, color='black')

        # Plot the bounds 
        self.plot_bounds(environment.bounds, color='black')

        self.ax.set_aspect('equal')
        plt.xlim(environment.bounds.min.x - 1, environment.bounds.max.x + 1)
        plt.ylim(environment.bounds.min.y - 1, environment.bounds.max.y + 1)
    
    def plot_pose(self, pose: Pose, **kwargs):
        """
        Display a pose on the plot
        
        Params:
            pose: The pose to plot
        """

        u = cos(pose.theta)
        v = sin(pose.theta)

        self.ax.quiver(pose.pos.x, pose.pos.y, u, v, **kwargs)
    
    def plot_poses(self, poses: Sequence[Pose], **kwargs):
        """
        Display a list of poses on the plot
        
        Params:
            poses: The poses to plot
        """
        # Create a list of xs and ys for plotting and thetas for calculating u and v
        x, y, theta = map(list[float], zip(*[(pose.pos.x, pose.pos.y, pose.theta) for pose in poses]))

        # Calculate the components of the arrows
        u, v = map(list[float], zip(*[(cos(t), sin(t)) for t in theta]))

        # Plot the arrows
        self.ax.quiver(x, y, u, v, **kwargs)
    
    def plot_bounds(self, bounds: Bounds, **kwargs):
        """
        Display a Bounds object on the plot
        
        Params:
            bounds: The bounds to plot
        """
        # Plot the bounds 
        bounds_x = [
            bounds.min.x,
            bounds.max.x,
            bounds.max.x,
            bounds.min.x,
            bounds.min.x,
        ]
        bounds_y = [
            bounds.min.y,
            bounds.min.y,
            bounds.max.y,
            bounds.max.y,
            bounds.min.y,
        ]
        self.ax.plot(bounds_x, bounds_y, **kwargs)

    def plot_vector(self, vector: Vector, **kwargs):
        """
        Display a vector on the plot

        Params:
            vector: the vector to plot
        """
        self.ax.plot(vector.x, vector.y, **kwargs)