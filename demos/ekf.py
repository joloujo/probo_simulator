from math import cos, sin, hypot, atan2, pi
import numpy as np
import sympy
from typing import Sequence

from probo_sim.environment import Environment
from probo_sim.robots import DifferentialDrive, DifferentialDriveState, DifferentialDriveControl
from probo_sim.sensors import GPS, Pinger, PingerState, PingerMeasurement
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

diff_start = DifferentialDriveState(
    Vector(1, 1)
)
diff_robot = DifferentialDrive(diff_start,
    variance=DifferentialDriveControl(PROCESS_LINEAR_VARIANCE, PROCESS_ANGULAR_VARIANCE)
)
diff_control: list[DifferentialDriveControl] = [
    DifferentialDriveControl(1, 0) ] * 30 + [ # Go forward three units
    DifferentialDriveControl(1, 1)] * 16 + [ # Turn left ~90 degrees
    DifferentialDriveControl(1, 0)] * 20 + [ # Go forward two units
    DifferentialDriveControl(1, -1)] * 16 + [# Turn right ~90 degrees
    DifferentialDriveControl(1, 0)] * 18 # Go forward a bit more

diff_ground_truth = GPS()
landmarks = [
    Vector(1, 9),
    Vector(3, 3),
    Vector(5, 7),
    Vector(9, 1),
    Vector(9, 9),
]
diff_pinger = Pinger(5, 0)

sim = Simulator(
    environment,
    [
        RobotControl(diff_robot, diff_control),
    ],
    [
        SensorState(diff_ground_truth, diff_robot),
        SensorState(diff_pinger, PingerState(diff_robot, landmarks)),
    ],
    DT
)

# Run the simulator
results = sim.run()

# EXTENDED KALMAN FILTER

def predict(x: DifferentialDriveState, P: np.ndarray, control: DifferentialDriveControl) -> tuple[DifferentialDriveState, np.ndarray]:
    x_hat: DifferentialDriveState = DifferentialDrive.kinematics(x, control, DT)

    if control.w == 0.0: 
        F = np.array([
            [1, 0, -DT * control.v * sin(x.theta)], 
            [0, 1, DT * control.v * cos(x.theta)], 
            [0, 0, 1]])
    else:
        F = np.array([
            [1, 0, control.v * (-cos(x.theta) + cos(DT * control.w + x.theta)) / control.w], 
            [0, 1, -control.v * (sin(x.theta) - sin(DT * control.w + x.theta)) / control.w], 
            [0, 0, 1]])
    
    process_noise = np.diag([PROCESS_LINEAR_VARIANCE, PROCESS_LINEAR_VARIANCE, PROCESS_ANGULAR_VARIANCE]) * DT**2

    P_hat = F @ P @ F.T + process_noise

    return x_hat, P_hat

def update(x_hat: DifferentialDriveState, P_hat: np.ndarray, measurement: PingerMeasurement) -> tuple[DifferentialDriveState, np.ndarray]:

    pings = measurement.pings

    for landmark, ping in zip(landmarks, pings):

        if ping is None:
            continue

        residual = ping - Vector.polar(
            hypot(landmark.x - x_hat.pos.x, landmark.y - x_hat.pos.y), 
            (atan2(landmark.y - x_hat.pos.y, landmark.x - x_hat.pos.x) - x_hat.theta + pi) % (2 * pi) - pi
        )

        x, y, theta = sympy.symbols('x y theta')

        f = sympy.Matrix([
            sympy.sqrt((landmark.x - x)**2 + (landmark.y - y)**2),
            sympy.atan2(sympy.sin(sympy.atan2(landmark.y - y, landmark.x - x) - theta), sympy.cos(sympy.atan2(landmark.y - y, landmark.x - x) - theta))
        ])
        J = f.jacobian([x, y, theta])
        H = np.array(J.evalf(subs={
            x: x_hat.pos.x,
            y: x_hat.pos.y,
            theta: x_hat.theta,
        })).astype(float)

        measurement_noise = np.diag([MEASUREMENT_LINEAR_VARIANCE, MEASUREMENT_ANGULAR_VARIANCE])

        S = H @ P_hat @ H.T + measurement_noise
        K = P_hat @ H.T @ np.linalg.inv(S)

        delta = K @ np.array([residual.x, residual.y])

        x_hat = DifferentialDriveState(
            Vector(
                x_hat.pos.x + delta[0], 
                x_hat.pos.y + delta[1]
            ), 
            x_hat.theta + delta[2])
        
        P_hat = P_hat - K @ H @ P_hat
    
    return x_hat, P_hat

def step(x: DifferentialDriveState, P: np.ndarray, control: DifferentialDriveControl, measurement: PingerMeasurement) -> tuple[DifferentialDriveState, np.ndarray]:
    x_hat, P_hat = predict(x, P, control)
    x, P = update(x_hat, P_hat, measurement)
    return x, P

def run(prior: DifferentialDriveState, initial_P: np.ndarray, controls: Sequence[DifferentialDriveControl], measurements: Sequence[PingerMeasurement]) -> tuple[list[DifferentialDriveState], list[np.ndarray]]:
    x: list[DifferentialDriveState] = []
    P: list[np.ndarray] = []

    steps = min(len(controls), len(measurements))

    for i in range(steps):
        previous_x = prior if i == 0 else x[i-1]
        previous_P = initial_P if i == 0 else P[i-1]

        next_x, next_P = step(previous_x, previous_P, controls[i], measurements[i])
        x.append(next_x)
        P.append(next_P)

    return x, P

xs, Ps = run(diff_start, np.zeros((3, 3)), diff_control, results[diff_pinger])

# Visualize the results
viz = Visualizer()

viz.plot_environment(environment)
viz.plot_vectors(landmarks, linestyle='None', marker='*', markersize=10, color='black')

viz.plot_poses([diff_start] + results[diff_ground_truth], alpha=0.5, color='green', label='Ground Truth')
viz.plot_poses([diff_start] + xs, alpha=0.5, color='red', label='Filtered Poses')

viz.legend(loc='upper right')

viz.show()
