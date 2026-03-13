import imageio.v2 as imageio
from math import pi
import numpy as np
import os
from typing import Literal

from probo_sim.environment import Environment
from probo_sim.graph_slam import GraphSLAM, OdomFactor, PingFactor, PriorFactor
from probo_sim.robots import DifferentialDrive, DifferentialDriveState, DifferentialDriveControl
from probo_sim.sensors import GPS, Pinger, PingerState
from probo_sim.simulator import Simulator, RobotControl, SensorState
from probo_sim.utils import Bounds, Vector, Pose
from probo_sim.visualizer import Visualizer

frames_folder = './tmp'
os.makedirs(frames_folder, exist_ok=True)
frames = []    

#region Set up and run the simulator
DT = 0.5  

environment = Environment(
    Bounds(Vector(-1, -3), Vector(7, 6)),
)

diff_start = DifferentialDriveState(Vector(1, 3))
diff_robot = DifferentialDrive(diff_start,
    DifferentialDriveControl(0.01, 0.01)
)
diff_control: list[DifferentialDriveControl] = [
    DifferentialDriveControl(1, 0) ] * round(3/DT) + [ # Go forward three units
    DifferentialDriveControl(pi/3, -pi/3)] * round(1.5/DT) + [ # Turn right 90 degrees
    DifferentialDriveControl(1, 0)] * round(3/DT) + [ # Go forward three units
    DifferentialDriveControl(pi/3, -pi/3)] * round(1.5/DT) + [ # Turn right 90 degrees
    DifferentialDriveControl(1, 0)] * round(2/DT) + [ # Go forward two units
    DifferentialDriveControl(pi/3, -pi/3)] * round(1.5/DT) + [ # Turn right 90 degrees
    DifferentialDriveControl(1, 0)] * round(3/DT) # Go forward three units

diff_gt = GPS()
diff_pinger = Pinger(3, 0, 
    # (0.001, 0.001)
)

gt_landmarks = [
    Vector(1, 4),
    Vector(1, 2),
    Vector(4, 4),
    Vector(5, -2),
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

gs.add_factor([0, 1, 2], PriorFactor(np.array([diff_start.pos.x, diff_start.pos.y, diff_start.theta])))

landmark_map: list[int | None] = [None] * len(gt_landmarks)
# landmark_map: list[int | None] = [None, None, None, None]

def n_landmarks() -> int:
    return len(landmark_map) - landmark_map.count(None)

n_poses = 1

def plot_and_save(i: int):

    viz = Visualizer()

    viz.plot_environment(environment)

    gs_landmarks: list[Vector] = []

    for i in range(n_landmarks()):
        start = 2*i
        gs_landmarks.append(Vector(gs.state[start], gs.state[start+1]))

    gs_poses: list[Pose] = []

    for i in range(n_poses):
        start = 2*n_landmarks() + 3*i
        gs_poses.append(Pose(Vector(gs.state[start], gs.state[start+1]), gs.state[start+2]))


    viz.plot_poses([diff_start] + results[diff_gt][:n_poses-1], color='red')
    viz.plot_vectors(gt_landmarks, linestyle='None', marker='*', ms=10, color='red')

    viz.plot_poses(gs_poses, color='blue')
    if len(gs_landmarks) > 0:
        viz.plot_vectors(gs_landmarks, linestyle='None', marker='*', ms=10, color='blue')

    filename = f'{frames_folder}/frame_{i:02d}.png'
    viz.save(filename)
    viz.close()

    frames.append(imageio.imread(filename))

gs.optimize()
plot_and_save(0)

for i, (control, pingerMeasurement) in enumerate(zip(diff_control, results[diff_pinger])):
    for j, ping in enumerate(pingerMeasurement.pings):
        if ping is None: continue

        if landmark_map[j] is None:

            gs.factors = [
                ([n + 2 if n >= n_landmarks() * 2 else n for n in factor[0]], factor[1]) 
                for factor in gs.factors
            ]

            landmark_map[j] = n_landmarks()
            gs.n += 2

        pose_start = n_landmarks() * 2 + (i + 1) * 3
        landmark_start: int = landmark_map[j] * 2 # type: ignore

        gs.add_factor(
            [pose_start, pose_start + 1, pose_start + 2] + [landmark_start, landmark_start + 1],
            PingFactor(np.array([ping.x, ping.y]), np.diag([0.0000000001, 0.0000000001]))
        )

    last_pose_start = n_landmarks() * 2 + i * 3
    delta = DifferentialDrive.kinematics(DifferentialDriveState(), control, DT)

    gs.n += 3
    n_poses += 1

    gs.add_factor(
        list(range(last_pose_start, last_pose_start+6)), 
        OdomFactor(np.array([delta.pos.x, delta.pos.y, delta.theta]), np.diag([0.01 * DT, 0.01**2 * DT, 0.01 * DT]))
    )

    gs.reset_state()

    gs.optimize()
    plot_and_save(i+1)

imageio.mimsave('media/animation.gif', frames, fps=5) # Save the frames as a GIF

# Clean up the temporary frame files
for filename in os.listdir(frames_folder):
    os.remove(os.path.join(frames_folder, filename))
os.rmdir(frames_folder)