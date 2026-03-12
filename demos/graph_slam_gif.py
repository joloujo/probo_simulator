from math import pi
import numpy as np
from typing import Literal

from probo_sim.environment import Environment
from probo_sim.graph_slam import GraphSLAM, OdomFactor, PingFactor, PriorFactor
from probo_sim.robots import DifferentialDrive, DifferentialDriveState, DifferentialDriveControl
from probo_sim.sensors import GPS, Pinger, PingerState
from probo_sim.simulator import Simulator, RobotControl, SensorState
from probo_sim.utils import Bounds, Vector, Pose
from probo_sim.visualizer import Visualizer

#region Set up and run the simulator
DT = 0.5  

environment = Environment(
    Bounds(Vector(0, 0), Vector(5, 5)),
)

diff_start = DifferentialDriveState(Vector(1, 3))
diff_robot = DifferentialDrive(diff_start,
    # DifferentialDriveControl(0.01, 0.01)
)
diff_control: list[DifferentialDriveControl] = [
    DifferentialDriveControl(1, 0) ] * round(1/DT) + [ # Go forward one unit
    DifferentialDriveControl(pi/3, -pi/3)] * round(1.5/DT) + [ # Turn right 90 degrees
    DifferentialDriveControl(1, 0)] * round(1/DT) # Go forward one unit

diff_gt = GPS()
diff_pinger = Pinger(3, 0, 
    # (0.001, 0.001)
)

gt_landmarks = [
    Vector(1, 4),
    Vector(4, 4),
    Vector(1, 2),
    Vector(4, 1),
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
# endregion

gs = GraphSLAM(3)

n_landmarks = 0
n_poses = 1

def plot():

    viz = Visualizer()

    viz.plot_environment(environment)

    gs_landmarks: list[Vector] = []

    for i in range(n_landmarks):
        start = 2*i
        gs_landmarks.append(Vector(gs.state[start], gs.state[start+1]))

    gs_poses: list[Pose] = []

    for i in range(n_poses):
        start = 2*n_landmarks + 3*i
        gs_poses.append(Pose(Vector(gs.state[start], gs.state[start+1]), gs.state[start+2]))


    viz.plot_poses([diff_start] + results[diff_gt][:n_poses-1], color='red')
    viz.plot_vectors(gt_landmarks, linestyle='None', marker='*', ms=10, color='red')

    viz.plot_poses(gs_poses, color='blue')
    if len(gs_landmarks) > 0:
        viz.plot_vectors(gs_landmarks, linestyle='None', marker='*', ms=10, color='blue')

    viz.show()

gs.add_factor([0, 1, 2], PriorFactor(np.array([diff_start.pos.x, diff_start.pos.y, diff_start.theta])))

gs.optimize()

plot()

gs.n += 7
n_landmarks = 2
n_poses = 2
gs.reset_state()

gs.factors = [([n + 4 for n in factor[0]], factor[1]) for factor in gs.factors]

gs.add_factor([4, 5, 6, 7, 8, 9], OdomFactor(np.array([0.5, 0, 0])))
gs.add_factor([7, 8, 9, 0, 1], PingFactor(np.array([-0.5, 1])))
gs.add_factor([7, 8, 9, 2, 3], PingFactor(np.array([-0.5, -1])))

gs.optimize()

plot()

exit()

n_poses = (1 + len(diff_control)) * 3

graph_slam_n = n_poses + len(gt_landmarks) * 2


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
    error = gs.gauss_newton_step()

    print(error)

    if last_error - error < 1e-6:
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

viz = Visualizer()

viz.plot_environment(environment)

viz.plot_poses([diff_start] + results[diff_gt], color='red')
viz.plot_vectors(gt_landmarks, linestyle='None', marker='*', ms=10, color='red')

viz.plot_poses(gs_poses, color='blue')
viz.plot_vectors(gs_landmarks, linestyle='None', marker='*', ms=10, color='blue')

viz.show()
