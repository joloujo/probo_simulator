from abc import ABC
from math import pi
import sympy

from probo_sim.environment import Environment
from probo_sim.robots import DifferentialDrive, DifferentialDriveState, DifferentialDriveControl, HolonomicDrive, HolonomicDriveState, HolonomicDriveControl
from probo_sim.sensors import GPS, Encoder, EncoderMeasurement, Pinger, PingerState
from probo_sim.simulator import Simulator, RobotControl, SensorState
from probo_sim.utils import Bounds, Vector, Pose
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


DT = 0.5

environment = Environment(
    Bounds(Vector(0, 0), Vector(5, 5)),
)

diff_start = DifferentialDriveState(
    Vector(1, 3)
)
diff_robot = DifferentialDrive(diff_start,
    DifferentialDriveControl(0.05, 0.05)
)
diff_control: list[DifferentialDriveControl] = [
    DifferentialDriveControl(1, 0) ] * 2 + [ # Go forward one unit
    DifferentialDriveControl(pi/3, -pi/3)] * 3 + [ # Turn right 90 degrees
    DifferentialDriveControl(1, 0)] * 2 # Go forward one unit


diff_gt = GPS()
diff_pinger = Pinger(3, 0, 
    (0.01, 0.01)
)

gt_landmarks = [
    Vector(1, 4),
    Vector(4, 4),
    Vector(1, 2)
]

sim = Simulator(
    environment,
    [
        RobotControl(diff_robot, diff_control),
    ],
    [
        SensorState(diff_gt, diff_robot),
        SensorState(diff_pinger, PingerState(diff_robot, gt_landmarks)),
    ],
    DT
)

# Run the simulator
results = sim.run()

# print(results[diff_pinger])

# viz = Visualizer()
# viz.plot_environment(environment)

# viz.plot_vectors(gt_landmarks, marker='*', ms=10, linestyle='None', color='black')

# viz.plot_poses([diff_start] + results[diff_gt], color='red')

# for pose, pingerMeasurement in zip(results[diff_gt], results[diff_pinger]):
#     for ping in pingerMeasurement.pings:
#         if ping is not None:
#             viz.plot_vectors([pose.pos, ping.rotate(pose.theta) + pose.pos], alpha=0.5, color='blue')

# viz.show()


# pose1 = Pose(Vector(0, 0), 0)
# pose2 = Pose(Vector(1, 0), 0)
# pose3 = Pose(Vector(2, -1), -pi/2)
# pose4 = Pose(Vector(2, -2), -pi/2)
# landmark1 = Vector(0, 1)
# landmark2 = Vector(3, 1)
# landmark3 = Vector(0, -1)

nodes: list[Pose | Vector] = \
    [Pose(Vector(0, 0), 0) for i in range(len(diff_control) + 1)] + \
    [Vector(0, 0) for i in range(len(gt_landmarks))]

factors: list[tuple[int, int, Pose | Vector]] = []

last_state = diff_start

for i, (control, pingerMeasurement) in enumerate(zip(diff_control, results[diff_pinger])):

    next_state = DifferentialDrive.kinematics(last_state, control, DT)

    delta_pose = Pose((next_state - last_state).pos.rotate(-last_state.theta), next_state.theta - last_state.theta)

    factors.append((i, i+1, delta_pose))

    for j, ping in enumerate(pingerMeasurement.pings):
        if ping is not None:
            factors.append((i+1, len(diff_control)+1+j, ping))

last_error = float('inf')

while True:
    gradient: list = \
        [Pose(Vector(0, 0), 0) for i in range(len(diff_control)+1)] + \
        [Vector(0, 0) for i in range(len(gt_landmarks))]

    error = 0

    for a, b, factor in factors:
        if isinstance(factor, Pose):
            error += OdomFactor(nodes[a], nodes[b], factor).squared_error # type: ignore
            delta1, delta2 = OdomFactor(nodes[a], nodes[b], factor).jacobian # type: ignore
            gradient[a] += delta1
            gradient[b] += delta2
        else: 
            error += PingFactor(nodes[a], nodes[b], factor).squared_error # type: ignore
            delta1, delta2 = PingFactor(nodes[a], nodes[b], factor).jacobian # type: ignore
            gradient[a] += delta1
            gradient[b] += delta2
    
    print(error)

    nodes = [node - 0.02 * g for node, g in zip(nodes, gradient)]

    if last_error - error < 0.001:
        break

    last_error = error


rotation = diff_start.theta - nodes[0].theta # type: ignore
translation: Vector = diff_start.pos - nodes[0].pos.rotate(rotation) # type: ignore

transformed_nodes: list[Pose | Vector] = []

for node in nodes:
    if isinstance(node, Pose):
        transformed_nodes.append(Pose(
            translation + node.pos.rotate(rotation),
            node.theta + rotation
        ))
    else: 
        transformed_nodes.append(
            translation + node.rotate(rotation),
        )

viz = Visualizer()

viz.plot_environment(environment)

viz.plot_poses([diff_start] + results[diff_gt], color='red')
viz.plot_vectors(gt_landmarks, linestyle='None', marker='*', ms=10, color='red')

for node in transformed_nodes:
    if isinstance(node, Pose):
        viz.plot_pose(node, color='blue')
    else: 
        viz.plot_vector(node, marker='*', ms=10, color='blue')

viz.show()
