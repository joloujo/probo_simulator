from itertools import product
import numpy as np
from math import cos, sin
from matplotlib.axes import Axes
from typing import Sequence

from probo_sim.environment import Environment
from probo_sim.utils import Bounds, Pose, Vector, Field

class Visualizer:
    """
    A class to visualize results from simulations
    """

    @classmethod
    def plot_environment(cls, axes: Axes, environment: Environment):
        """
        Display an environment on the plot
        
        Params:
            environment: The environment to plot
        """
        # Plot each obstacle
        for obstacle in environment.obstacles:
            Visualizer.plot_bounds(axes, obstacle, color='black')

        # Plot the bounds 
        Visualizer.plot_bounds(axes, environment.bounds, color='black')

        axes.set_aspect('equal')
        axes.set_xlim(left=environment.bounds.min.x - 1, right=environment.bounds.max.x + 1)
        axes.set_ylim(bottom=environment.bounds.min.y - 1, top=environment.bounds.max.y + 1)
    
    @classmethod
    def plot_pose(cls, axes: Axes, pose: Pose, **kwargs):
        """
        Display a pose on the plot
        
        Params:
            pose: The pose to plot
        """

        u = cos(pose.theta)
        v = sin(pose.theta)

        axes.quiver(pose.pos.x, pose.pos.y, u, v, **kwargs)
    
    @classmethod
    def plot_poses(cls, axes: Axes, poses: Sequence[Pose], **kwargs):
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
        axes.quiver(x, y, u, v, **kwargs)
    
    @classmethod
    def plot_bounds(cls, axes: Axes, bounds: Bounds, **kwargs):
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

        axes.plot(bounds_x, bounds_y, **kwargs)

    @classmethod
    def plot_vector(cls, axes: Axes, vector: Vector, inset: bool = False, **kwargs):
        """
        Display a vector on the plot

        Params:
            vector: the vector to plot
        """

        axes.plot(vector.x, vector.y, **kwargs)

    @classmethod
    def plot_vectors(cls, axes: Axes, vectors: Sequence[Vector], inset: bool = False, **kwargs):
        """
        Display a vector on the plot

        Params:
            vector: the vector to plot
        """

        x, y = zip(*[(v.x, v.y) for v in vectors])

        axes.plot(x, y, **kwargs)
    
    @classmethod
    def plot_field(cls, axes: Axes, field: Field, bounds: Bounds, count: tuple[int, int] = (11, 11), inset: bool = False, **kwargs):

        x = np.linspace(bounds.min.x, bounds.max.x, count[0])
        y = np.linspace(bounds.min.y, bounds.max.y, count[1])
        z = np.array([field(Vector(x, y)) for x, y in product(x, y)])

        Visualizer.plot_field_values(axes, x, y, z, inset, **kwargs)
    
    @classmethod
    def plot_field_values(cls, axes: Axes, x: np.ndarray, y: np.ndarray, z: np.ndarray, inset: bool = False, **kwargs):
        X, Y = np.meshgrid(x, y)
        axes.contourf(X, Y, z.reshape(len(y), len(x)).T, **kwargs)