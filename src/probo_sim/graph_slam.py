from abc import ABC, abstractmethod
from math import cos, sin, sqrt
import sympy
from typing import Any
from pprint import pp

from probo_sim.utils import Pose, Vector, Bounds
from probo_sim.visualizer import Visualizer

class Factor[A, B](ABC):
    pass

class OdomFactor(Factor[Pose, Pose]):
    def __init__(self, a: Pose, b: Pose, measurement: Pose) -> None:
        self.a = a
        self.b = b

        # Define error and calculate gradient and jacobian once for computational efficiency

        x_a, y_a, t_a, x_b, y_b, t_b = sympy.symbols('x_a, y_a, t_a, x_b, y_b, t_b')
        variables = sympy.Matrix([x_a, y_a, t_a, x_b, y_b, t_b])

        self._squared_error = \
            (sympy.cos(t_a) * (x_b - x_a) + sympy.sin(t_a) * (y_b - y_a) - measurement.pos.x) ** 2 + \
            (-1 * sympy.sin(t_a) * (x_b - x_a) + sympy.cos(t_a) * (y_b - y_a) - measurement.pos.y) ** 2 + \
            sympy.atan2(sympy.sin(t_b - t_a - measurement.theta), sympy.cos(t_b - t_a - measurement.theta)) ** 2

        self._jacobian = sympy.Matrix([self._squared_error]).jacobian(variables)

    @property
    def squared_error(self) -> float:

        x_a, y_a, t_a, x_b, y_b, t_b = sympy.symbols('x_a, y_a, t_a, x_b, y_b, t_b')

        return self._squared_error.subs({
            x_a: self.a.pos.x, 
            y_a: self.a.pos.y, 
            t_a: self.a.theta, 
            x_b: self.b.pos.x, 
            y_b: self.b.pos.y, 
            t_b: self.b.theta
        }).evalf()
    
    @property
    def jacobian(self) -> tuple[Pose, Pose]:
        x_a, y_a, t_a, x_b, y_b, t_b = sympy.symbols('x_a, y_a, t_a, x_b, y_b, t_b')

        jacobian = self._jacobian.subs({
            x_a: self.a.pos.x, 
            y_a: self.a.pos.y, 
            t_a: self.a.theta, 
            x_b: self.b.pos.x, 
            y_b: self.b.pos.y, 
            t_b: self.b.theta
        }).evalf()

        return (Pose(
            Vector(jacobian[0], jacobian[1]),
            jacobian[2]
        ), Pose(
            Vector(jacobian[3], jacobian[4]),
            jacobian[5]
        ))

class PingFactor(Factor[Pose, Vector]):
    def __init__(self, a: Pose, b: Vector, measurement: Vector) -> None:
        self.a = a
        self.b = b

        x_a, y_a, t_a, x_b, y_b = sympy.symbols('x_a, y_a, t_a, x_b, y_b')
        variables = sympy.Matrix([x_a, y_a, t_a, x_b, y_b])

        self._squared_error = \
            (sympy.cos(t_a) * (x_b - x_a) + sympy.sin(t_a) * (y_b - y_a) - measurement.x) ** 2 + \
            (-1 * sympy.sin(t_a) * (x_b - x_a) + sympy.cos(t_a) * (y_b - y_a) - measurement.y) ** 2
            
        self._jacobian = sympy.Matrix([self._squared_error]).jacobian(variables)

    @property
    def squared_error(self) -> float:

        x_a, y_a, t_a, x_b, y_b = sympy.symbols('x_a, y_a, t_a, x_b, y_b')

        return self._squared_error.subs({
            x_a: self.a.pos.x, 
            y_a: self.a.pos.y, 
            t_a: self.a.theta, 
            x_b: self.b.x, 
            y_b: self.b.y, 
        }).evalf()
    
    @property
    def jacobian(self) -> tuple[Pose, Vector]:
        x_a, y_a, t_a, x_b, y_b = sympy.symbols('x_a, y_a, t_a, x_b, y_b')

        jacobian = self._jacobian.subs({
            x_a: self.a.pos.x, 
            y_a: self.a.pos.y, 
            t_a: self.a.theta, 
            x_b: self.b.x, 
            y_b: self.b.y, 
        }).evalf()

        return (Pose(
            Vector(jacobian[0], jacobian[1]),
            jacobian[2]
        ),
        Vector(jacobian[3], jacobian[4]))








pose1 = Pose(Vector(0, 0), 0)
pose2 = Pose(Vector(0, 0), 0)
pose3 = Pose(Vector(0, 0), 0)
pose4 = Pose(Vector(0, 0), 0)
landmark1 = Vector(0, 0)
landmark2 = Vector(0, 0)

print(pose1, pose2)
for i in range(100):
    pose1delta1, pose2delta1 = OdomFactor(pose1, pose2, Pose(Vector(1, 0), 0)).jacobian
    pose1delta2, landmark1delta1 = PingFactor(pose1, landmark1, Vector(0, 1)).jacobian
    pose2delta2, landmark1delta2 = PingFactor(pose2, landmark1, Vector(-1, 1)).jacobian

    pose1 -= 0.01 * (pose1delta1 + pose1delta2)
    pose2 -= 0.01 * (pose2delta1 + pose2delta2)
    landmark1 -= 0.01 * (landmark1delta1 + landmark1delta2)

    print(pose1, pose2, landmark1)

viz = Visualizer()


viz.plot_bounds(Bounds(Vector(-1, -1), Vector(2, 2)))
viz.plot_poses([pose1, pose2])
viz.plot_vectors([landmark1], marker='x')

viz.show()
