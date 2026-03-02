import numpy as np

from probo_sim.environment import Environment
from probo_sim.kalman_filter import KalmanFilter
from probo_sim.robots import HolonomicDrive, HolonomicDriveState, HolonomicDriveControl
from probo_sim.sensors import GPS
from probo_sim.simulator import Simulator, RobotControl, SensorState
from probo_sim.utils import Bounds, Vector, Pose
from probo_sim.visualizer import Visualizer

PROCESS_LINEAR_VARIANCE = 0.1
PROCESS_ANGULAR_VARIANCE = 0.1
MEASUREMENT_LINEAR_VARIANCE = 0.1
MEASUREMENT_ANGULAR_VARIANCE = 0.1

DT = 0.1

environment = Environment(
    Bounds(Vector(0, 0), Vector(10, 10)),
    [
        Bounds(Vector(0, 2), Vector(4, 4)),
        Bounds(Vector(2, 6), Vector(4, 8)),
        Bounds(Vector(6, 0), Vector(8, 4)),
    ],
)

holo_start = HolonomicDriveState(
    Vector(1, 1)
)
holo_robot = HolonomicDrive(holo_start,
    variance=HolonomicDriveControl(PROCESS_LINEAR_VARIANCE, PROCESS_LINEAR_VARIANCE, PROCESS_ANGULAR_VARIANCE)
)
holo_control: list[HolonomicDriveControl] = [
    HolonomicDriveControl(1, 0, 0)] * 40 + [ # Go right four units
    HolonomicDriveControl(0, 1, 31.415/20)] * 40 + [ # Go up four units
    HolonomicDriveControl(-1, 0, 0)] * 20 # Go left two units

holo_ground_truth = GPS()
holo_gps = GPS(
    variance=Pose(Vector(MEASUREMENT_LINEAR_VARIANCE, MEASUREMENT_LINEAR_VARIANCE), MEASUREMENT_ANGULAR_VARIANCE)
)

sim = Simulator(
    environment,
    [
        RobotControl(holo_robot, holo_control),
    ],
    [
        SensorState(holo_ground_truth, holo_robot),
        SensorState(holo_gps, holo_robot),
    ],
    DT
)

kalman_filter = KalmanFilter(
    transition_matrix=np.identity(3),
    control_matrix=np.identity(3) * DT,
    measurement_matrix=np.identity(3),
    process_noise=np.array([
        [PROCESS_LINEAR_VARIANCE * DT**2, 0, 0],
        [0, PROCESS_LINEAR_VARIANCE * DT**2, 0],
        [0, 0, PROCESS_ANGULAR_VARIANCE * DT**2],
    ]),
    measurement_noise=np.array([
        [MEASUREMENT_LINEAR_VARIANCE, 0, 0],
        [0, MEASUREMENT_LINEAR_VARIANCE, 0],
        [0, 0, MEASUREMENT_ANGULAR_VARIANCE],
    ])
)

# Run the simulator
results = sim.run()

prior = np.array([holo_start.pos.x, holo_start.pos.y, holo_start.theta])
initial_P = np.zeros((3, 3))

kalman_filter_controls = [
    np.array([control.xv, control.yv, control.w])
    for control in holo_control
]

kalman_filter_measurements = [
    np.array([measurement.pos.x, measurement.pos.y, measurement.theta])
    for measurement in results[holo_gps]
]

xs, Ps = kalman_filter.run(
    prior,
    initial_P,
    kalman_filter_controls,
    kalman_filter_measurements
)

poses = [
    Pose(Vector(x[0], x[1]), x[2])
    for x in xs
]

# Visualize the results
viz = Visualizer()

viz.plot_environment(environment)

viz.plot_poses([holo_start] + results[holo_ground_truth], alpha=0.5, color='green', label='Ground Truth')
viz.plot_poses([holo_start] + results[holo_gps], alpha=0.5, color='blue', label='GPS Measurements')
viz.plot_poses([holo_start] + poses, alpha=0.5, color='red', label='Filtered Poses')

viz.legend(loc='upper right')

viz.show()
