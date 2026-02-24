from math import cos, sin
import matplotlib.pyplot as plt

from environment import Environment
from utils import Bounds, Pose, Vector

class Visualizer:
    def __init__(self) -> None:
        self.fig, self.ax = plt.subplots()

    def show(self):
        plt.show()

    def plot_environment(self, environment: Environment):
        # Plot each obstacle
        for obstacle in environment.obstacles:
            self.plot_bounds(obstacle, color='black')

        # Plot the bounds 
        self.plot_bounds(environment.bounds, color='black')

        self.ax.set_aspect('equal')
        plt.xlim(environment.bounds.min.x - 1, environment.bounds.max.x + 1)
        plt.ylim(environment.bounds.min.y - 1, environment.bounds.max.y + 1)
    
    def plot_poses(self, poses: list[Pose], **kwargs):
        x: list[float]
        y: list[float]
        theta: list[float]
        x, y, theta = map(list, zip(*[(pose.pos.x, pose.pos.y, pose.theta) for pose in poses]))

        u: list[float]
        v: list[float]
        u, v = map(list, zip(*[(cos(t), sin(t)) for t in theta]))

        self.ax.quiver(x, y, u, v, **kwargs)
    
    def plot_bounds(self, bounds: Bounds, **kwargs):
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
        self.ax.plot(vector.x, vector.y, **kwargs)