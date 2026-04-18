from itertools import product
import numpy as np
from math import cos, sin
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from typing import Sequence

from probo_sim.environment import Environment
from probo_sim.utils import Bounds, Pose, Vector, Field

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

        self._ax_inset: Axes | None = None

    @property
    def ax_inset(self) -> Axes:
        if self._ax_inset is None:
            self._ax_inset = self.ax.inset_axes((0.85, 0.15, 0.3, 0.3))
    
        return self._ax_inset

    def show(self):
        """
        Show the plot
        """
        plt.show()

    def save(self, filename: str):
        plt.savefig(filename)

    def close(self):
        plt.close(self.fig)

    def legend(self, *args, **kwargs):
        """
        Show the plot
        """
        self.ax.legend(*args, **kwargs)

    def plot_environment(self, environment: Environment, inset: bool = False):
        """
        Display an environment on the plot
        
        Params:
            environment: The environment to plot
        """
        # Plot each obstacle
        for obstacle in environment.obstacles:
            self.plot_bounds(obstacle, inset, color='black')

        # Plot the bounds 
        self.plot_bounds(environment.bounds, inset, color='black')

        axes = self.ax_inset if inset else self.ax

        axes.set_aspect('equal')
        plt.xlim(environment.bounds.min.x - 1, environment.bounds.max.x + 1)
        plt.ylim(environment.bounds.min.y - 1, environment.bounds.max.y + 1)
    
    def plot_pose(self, pose: Pose, inset: bool = False, **kwargs):
        """
        Display a pose on the plot
        
        Params:
            pose: The pose to plot
        """

        u = cos(pose.theta)
        v = sin(pose.theta)

        axes = self.ax_inset if inset else self.ax

        axes.quiver(pose.pos.x, pose.pos.y, u, v, **kwargs)
    
    def plot_poses(self, poses: Sequence[Pose], inset: bool = False, **kwargs):
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
        axes = self.ax_inset if inset else self.ax

        axes.quiver(x, y, u, v, **kwargs)
    
    def plot_bounds(self, bounds: Bounds, inset: bool = False, **kwargs):
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

        axes = self.ax_inset if inset else self.ax

        axes.plot(bounds_x, bounds_y, **kwargs)

    def plot_vector(self, vector: Vector, inset: bool = False, **kwargs):
        """
        Display a vector on the plot

        Params:
            vector: the vector to plot
        """
        axes = self.ax_inset if inset else self.ax

        axes.plot(vector.x, vector.y, **kwargs)

    def plot_vectors(self, vectors: Sequence[Vector], inset: bool = False, **kwargs):
        """
        Display a vector on the plot

        Params:
            vector: the vector to plot
        """

        x, y = zip(*[(v.x, v.y) for v in vectors])

        axes = self.ax_inset if inset else self.ax

        axes.plot(x, y, **kwargs)
    
    def plot_field(self, field: Field, bounds: Bounds, count: tuple[int, int] = (11, 11), inset: bool = False, **kwargs):

        x = np.linspace(bounds.min.x, bounds.max.x, count[0])
        y = np.linspace(bounds.min.y, bounds.max.y, count[1])
        z = np.array([field(Vector(x, y)) for x, y in product(x, y)])

        self.plot_field_values(x, y, z, inset, **kwargs)
    
    def plot_field_values(self, x: np.ndarray, y: np.ndarray, z: np.ndarray, inset: bool = False, **kwargs):
        X, Y = np.meshgrid(x, y)

        axes = self.ax_inset if inset else self.ax

        axes.contourf(X, Y, z.reshape(len(y), len(x)).T, **kwargs)