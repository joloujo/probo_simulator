from abc import ABC, abstractmethod
from math import cos, sin, sqrt
import sympy
from typing import Any
from pprint import pp

from probo_sim.utils import Pose, Vector, Bounds
from probo_sim.visualizer import Visualizer

class Factor[A, B](ABC):
    @abstractmethod
    def squared_error(self) -> float:
        pass

    # @abstractmethod
    # def gradient(self) -> tuple[A, B]:
    #     pass

class OdomFactor(Factor[Pose, Pose]):
    def __init__(self, a: Pose, b: Pose, measurement: Pose) -> None:
        self.a = a
        self.b = b
        self.m = measurement

    def squared_error(self) -> float:
        expected = self.b - self.a

        print(f'{expected = }')
        print(f'{self.m = }')

        diff = expected - self.m

        print(f'{diff = }')

        return diff.pos.x**2 + diff.pos.y**2 + diff.theta**2
    
    def gradient(self) -> tuple[Pose, Pose]:
        return (
            -2 * (self.b - self.a - self.m),
            2 * (self.b - self.a - self.m)
        )
        # return (
        #     Pose(
        #         Vector(
        #             2 * self.a.pos.x + 2 * (self.m.pos.x - self.b.pos.x),
        #             2 * self.a.pos.y + 2 * (self.m.pos.y - self.b.pos.y)
        #         ),
        #         2 * self.a.theta + 2 * (self.m.theta - self.b.theta)
        #     ),
        #     Pose(
        #         Vector(
        #             2 * self.b.pos.x - 2 * (self.a.pos.x + self.m.pos.x),
        #             2 * self.b.pos.y - 2 * (self.a.pos.y + self.m.pos.y)
        #         ),
        #         2 * self.b.theta - 2 * (self.a.theta + self.m.theta)
        #     )
        # )

class PingFactor(Factor[Pose, Vector]):
    def __init__(self, a: Pose, b: Vector, measurement: Vector) -> None:
        self.a = a
        self.b = b
        self.m = measurement

    def squared_error(self) -> float:
        expected = (self.b - self.a.pos).rotate(-self.a.theta)

        print(f'{expected = }')
        print(f'{self.m = }')

        diff = expected - self.m

        print(f'{diff = }')

        return diff.x**2 + diff.y**2
    
    def gradient(self) -> tuple[Pose, Vector]:
        return (
            Pose(
                Vector(
                    -2*((-self.a.pos.x + self.b.x)*cos(self.a.theta) + (-self.a.pos.y + self.b.y)*sin(self.a.theta))*cos(self.a.theta) + 2*(-(-self.a.pos.x + self.b.x)*sin(self.a.theta) + (-self.a.pos.y + self.b.y)*cos(self.a.theta) - 1)*sin(self.a.theta),
                    -2*((-self.a.pos.x + self.b.x)*cos(self.a.theta) + (-self.a.pos.y + self.b.y)*sin(self.a.theta))*sin(self.a.theta) - 2*(-(-self.a.pos.x + self.b.x)*sin(self.a.theta) + (-self.a.pos.y + self.b.y)*cos(self.a.theta) - 1)*cos(self.a.theta)
                ),
                (-2*(-self.a.pos.x + self.b.x)*sin(self.a.theta) + 2*(-self.a.pos.y + self.b.y)*cos(self.a.theta))*((-self.a.pos.x + self.b.x)*cos(self.a.theta) + (-self.a.pos.y + self.b.y)*sin(self.a.theta)) + (-2*(-self.a.pos.x + self.b.x)*cos(self.a.theta) - 2*(-self.a.pos.y + self.b.y)*sin(self.a.theta))*(-(-self.a.pos.x + self.b.x)*sin(self.a.theta) + (-self.a.pos.y + self.b.y)*cos(self.a.theta) - 1)
            ),
            Vector(
                2*((-self.a.pos.x + self.b.x)*cos(self.a.theta) + (-self.a.pos.y + self.b.y)*sin(self.a.theta))*cos(self.a.theta) - 2*(-(-self.a.pos.x + self.b.x)*sin(self.a.theta) + (-self.a.pos.y + self.b.y)*cos(self.a.theta) - 1)*sin(self.a.theta),
                2*((-self.a.pos.x + self.b.x)*cos(self.a.theta) + (-self.a.pos.y + self.b.y)*sin(self.a.theta))*sin(self.a.theta) + 2*(-(-self.a.pos.x + self.b.x)*sin(self.a.theta) + (-self.a.pos.y + self.b.y)*cos(self.a.theta) - 1)*cos(self.a.theta)
            )
        )


# class GraphSLAM():
#     nodes: list[Any]
#     factors: list[Factor]
#     last_given_pose: Pose

#     def __init__(self) -> None:
#         self.nodes = []
#         self.factors = []




# pose1 = Pose(Vector(1, 1), 0)
# pose2 = Pose(Vector(2, 1), 0)
# ladmark1 = Vector(1, 2)

pose1 = Pose(Vector(0, 0), 0)
pose2 = Pose(Vector(0, 0), 0)
landmark1 = Vector(0, 0)

factors: list[Factor] = [
    OdomFactor(pose1, pose2, Pose(Vector(1, 0), 0)),
    PingFactor(pose1, landmark1, Vector(0, 1)),
    PingFactor(pose2, landmark1, Vector(-1, 1)),
]

# for factor in factors:
#     print(factor.squared_error())


print(pose1, pose2)
for i in range(1000):
    pose1delta1, pose2delta1 = OdomFactor(pose1, pose2, Pose(Vector(1, 0), 0)).gradient()
    pose1delta2, landmark1delta1 = PingFactor(pose1, landmark1, Vector(0, 1)).gradient()
    pose2delta2, landmark1delta2 = PingFactor(pose2, landmark1, Vector(-1, 1)).gradient()

    pose1 -= 0.01 * (pose1delta1 + pose1delta2)
    pose2 -= 0.01 * (pose2delta1 + pose2delta2)
    landmark1 -= 0.01 * (landmark1delta1 + landmark1delta2)

    print(pose1, pose2, landmark1)

viz = Visualizer()

viz.plot_bounds(Bounds(Vector(-1, -1), Vector(2, 2)))
viz.plot_poses([pose1, pose2])
viz.plot_vectors([landmark1], marker='x')

viz.show()

# gs = GraphSLAM(Pose(Vector(1, 1), 0))

# gs.nodes[-1] += Pose(Vector(1, 0), 1)

# gs.step(Pose(Vector(1, 2), 0), [])

# print(gs.nodes)