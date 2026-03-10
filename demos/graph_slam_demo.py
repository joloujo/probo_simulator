from math import pi
import numpy as np

from probo_sim.environment import Environment
from probo_sim.graph_slam_2 import GraphSLAM, OdomFactor, PingFactor
from probo_sim.robots import DifferentialDrive, DifferentialDriveState, DifferentialDriveControl
from probo_sim.sensors import GPS, Pinger, PingerState
from probo_sim.simulator import Simulator, RobotControl, SensorState
from probo_sim.utils import Bounds, Vector, Pose
from probo_sim.visualizer import Visualizer

DT = 0.5

environment = Environment(
    Bounds(Vector(0, 0), Vector(5, 5)),
)

diff_start = DifferentialDriveState(Vector(1, 3))
diff_robot = DifferentialDrive(diff_start,
    # DifferentialDriveControl(0.05, 0.05)
)
diff_control: list[DifferentialDriveControl] = [
    DifferentialDriveControl(1, 0) ] * round(1/DT) + [ # Go forward one unit
    DifferentialDriveControl(pi/3, -pi/3)] * round(1.5/DT) + [ # Turn right 90 degrees
    DifferentialDriveControl(1, 0)] * round(1/DT) # Go forward one unit

diff_gt = GPS()
diff_pinger = Pinger(3, 0, 
    # (0.01, 0.01)
)

gt_landmarks = [
    Vector(1, 4),
    Vector(4, 4),
    Vector(1, 2),
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

print("Sim done, adding factors")

n_poses = (1 + len(diff_control)) * 3

graph_slam_n = n_poses + len(gt_landmarks) * 2

gs = GraphSLAM(graph_slam_n)

for i, control in enumerate(diff_control):
    indexes = list(range(i*3, (i+2)*3))
    delta = DifferentialDrive.kinematics(DifferentialDriveState(), control, DT)
    factor = OdomFactor(np.array([delta.pos.x, delta.pos.y, delta.theta]))

    gs.add_factor(indexes, factor)

for i, pingerMeasurement in enumerate(results[diff_pinger]):
    for j, ping in enumerate(pingerMeasurement.pings):
        if ping is None: continue

        indexes = list(range((i+1)*3, (i+2)*3)) + list(range(n_poses + j*2, n_poses + (j+1)*2))
        factor = PingFactor(np.array([ping.x, ping.y]))

        gs.add_factor(indexes, factor)

print('Optimizing')

last_error = float('inf')

while True:
    error = gs.gradient_descent_step(0.01)

    if last_error - error < 0.0001:
        break

    last_error = error

print(f'Done with error {last_error}')

gs_poses: list[Pose] = []

for n in range(len(diff_control) + 1):
    i = n*3
    gs_poses.append(Pose(Vector(gs.state[i], gs.state[i+1]), gs.state[i+2]))

gs_landmarks: list[Vector] = []

for n in range(len(gt_landmarks)):
    i = n_poses + n*2
    gs_landmarks.append(Vector(gs.state[i], gs.state[i+1]))

rotation = diff_start.theta - gs_poses[0].theta # type: ignore
translation: Vector = diff_start.pos - gs_poses[0].pos.rotate(rotation) # type: ignore

transformed_gs_poses: list[Pose] = []

for pose in gs_poses:
    transformed_gs_poses.append(Pose(
        translation + pose.pos.rotate(rotation),
        pose.theta + rotation
    ))

transformed_gs_landmarks: list[Vector] = []

for landmark in gs_landmarks:
    transformed_gs_landmarks.append(
        translation + landmark.rotate(rotation),
    )

viz = Visualizer()

viz.plot_environment(environment)

viz.plot_poses([diff_start] + results[diff_gt], color='red')
viz.plot_vectors(gt_landmarks, linestyle='None', marker='*', ms=10, color='red')

viz.plot_poses(transformed_gs_poses, color='blue')
viz.plot_vectors(transformed_gs_landmarks, linestyle='None', marker='*', ms=10, color='blue')

viz.show()
