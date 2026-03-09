from abc import ABC, abstractmethod
from math import cos, sin, sqrt, pi
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



# pose1 = Pose(Vector(0, 0), 0)
# pose2 = Pose(Vector(1, 0), 0)
# pose3 = Pose(Vector(2, -1), -pi/2)
# pose4 = Pose(Vector(2, -2), -pi/2)
# landmark1 = Vector(0, 1)
# landmark2 = Vector(3, 1)
# landmark3 = Vector(0, -1)

nodes: list[Pose | Vector] = [
    Pose(Vector(0, 0), 0),
    Pose(Vector(0, 0), 0),
    Pose(Vector(0, 0), 0),
    Pose(Vector(0, 0), 0),
    Vector(0, 0),
    Vector(0, 0),
    Vector(0, 0)
]

factors: list[tuple[int, int, Pose | Vector]] = [
    (0, 1, Pose(Vector(1, 0), 0)),
    (1, 2, Pose(Vector(1, -1), -pi/2)),
    (2, 3, Pose(Vector(1, 0), 0)),
    (0, 4, Vector(0, 1)),
    (0, 6, Vector(0, -1)),
    (1, 4, Vector(-1, 1)),
    (1, 5, Vector(2, 1)),
    (1, 6, Vector(-1, -1)),
    (2, 5, Vector(-2, 1)),
    (2, 6, Vector(0, -2)),
    (3, 6, Vector(-1, -2)),
]

for i in range(100):
    gradient = [
        Pose(Vector(0, 0), 0),
        Pose(Vector(0, 0), 0),
        Pose(Vector(0, 0), 0),
        Pose(Vector(0, 0), 0),
        Vector(0, 0),
        Vector(0, 0),
        Vector(0, 0)
    ]

    for a, b, factor in factors:
        if isinstance(factor, Pose):
            delta1, delta2 = OdomFactor(nodes[a], nodes[b], factor).jacobian # type: ignore
            gradient[a] += delta1
            gradient[b] += delta2
        else: 
            delta1, delta2 = PingFactor(nodes[a], nodes[b], factor).jacobian # type: ignore
            gradient[a] += delta1
            gradient[b] += delta2
    
    nodes = [node - 0.01 * g for node, g in zip(nodes, gradient)]
    

viz = Visualizer()


viz.plot_bounds(Bounds(Vector(-1, -3), Vector(4, 2)))

for node in nodes:
        if isinstance(node, Pose):
            viz.plot_pose(node)
        else: 
            viz.plot_vector(node, marker='x')

viz.show()
