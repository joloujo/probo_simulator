from itertools import product
import matplotlib.pyplot as plt
from math import sqrt
from matplotlib.axes import Axes
import numpy as np
import random
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF

from probo_sim.environment import Environment
from probo_sim.robots import DifferentialDrive, DifferentialDriveState, DifferentialDriveControl
from probo_sim.sensors import GPS, Encoder, InsituInstrument, InsituInstrumentState
from probo_sim.simulator import Simulator, RobotControl, SensorState, open_loop
from probo_sim.utils import Bounds, Vector, gaussian
from probo_sim.visualizer import Visualizer

DT = 0.5
SENSOR_PERIOD = 1
VARIANCE = 0
LENGTH_SCALE = 2.5
B = 1

def value_field(position: Vector) -> float:

    point = np.array([position.x, position.y])

    return sum([
        gaussian(point, 1, np.array([8, 8]), np.array([2])),
        gaussian(point, 0.6, np.array([2, 5]), np.array([3, 2])),
        gaussian(point, 0.8, np.array([9, 1]), np.array([3]))
    ])


environment = Environment(
    Bounds(Vector(0, 0), Vector(10, 10)),
    [
        # Bounds(Vector(0, 2), Vector(4, 4)),
        # Bounds(Vector(2, 6), Vector(4, 8)),
        # Bounds(Vector(6, 0), Vector(8, 4)),
    ],
)

robot_start = DifferentialDriveState(
    Vector(1, 1.1)
)
robot = DifferentialDrive(robot_start)

gps = GPS()
odom = Encoder(0)
field_sensor = InsituInstrument(SENSOR_PERIOD, VARIANCE)
field_sensor_locations = GPS(SENSOR_PERIOD)

action_library: list[DifferentialDriveControl] = [
    DifferentialDriveControl(v, w)
    for (v, w) in product(np.linspace(0., 1., 3), np.linspace(-2, 2, 9))
]

# Calculate belief
belief = GaussianProcessRegressor(
    kernel = RBF([LENGTH_SCALE, LENGTH_SCALE], 'fixed'),
)

# Visualize the results
fig, ax = plt.subplots(2, 2)

def controller(sim: Simulator) -> DifferentialDriveControl | None:

    if sim.i == 0:
        return DifferentialDriveControl(1, 0)
    
    if not plt.fignum_exists(fig.number):
        return None

    if sim.i == 1000:
        return None

    belief.fit(
        np.asarray([(pose.pos.x, pose.pos.y) for pose in sim.results[field_sensor_locations]]), 
        np.asarray(sim.results[field_sensor])
    )

    # Determine the reward for each action

    current_pose = sim.results[gps][-1]
    best_rewards: list[float] = []
    total_rewards: list[float] = []

    for action in action_library:

        best_reward = float('-inf')
        total_reward = 0.

        for _ in range(100):

            next_pose = DifferentialDrive.kinematics(
                DifferentialDriveState(current_pose.pos, current_pose.theta), 
                action,
                DT,
            )
            
            for _ in range(20):
                next_pose = DifferentialDrive.kinematics(
                    DifferentialDriveState(next_pose.pos, next_pose.theta), 
                    random.choice(action_library),
                    DT,
                )
            
            mean, stddev = belief.predict([(next_pose.pos.x, next_pose.pos.x)], return_std=True) # type: ignore

            reward = float((mean + sqrt(B) * stddev)[0])

            total_reward += reward
            best_reward = max(reward, best_reward)
        
        best_rewards.append(best_reward)
        total_rewards.append(total_reward)

    best_action = action_library[np.argmax(best_rewards)]
    # best_action = action_library[np.argmax(total_rewards)]

    return best_action

ground_truth_plot: Axes = ax[0, 0]
belief_plot: Axes = ax[1, 0]
uncertainty_plot: Axes = ax[1, 1]
planning_plot: Axes = ax[0, 1]

def plot(sim: Simulator):
    for axis in ax.flatten():
        Visualizer.plot_environment(axis, environment)
        axis.clear()

    ground_truth_plot.text(0, 1.05, f'Time: {sim.time}', transform=ground_truth_plot.transAxes)

    x = np.linspace(environment.bounds.min.x, environment.bounds.max.x, 21)
    y = np.linspace(environment.bounds.min.y, environment.bounds.max.y, 21)
    M = np.array(list(product(x,y)))
    c_sample, std_dev = belief.predict(M, return_std=True) # type: ignore

    reward = c_sample + sqrt(B) * std_dev

    Visualizer.plot_field(ground_truth_plot, value_field, environment.bounds, levels=np.linspace(0, 1.1, 23))
    Visualizer.plot_field_values(belief_plot, x, y, c_sample, levels=np.linspace(0, 1.1, 23))
    Visualizer.plot_field_values(uncertainty_plot, x, y, std_dev, levels=20)
    Visualizer.plot_field_values(planning_plot, x, y, reward, levels=20)

    Visualizer.plot_poses(ground_truth_plot, [robot_start] + sim.results[gps], alpha=0.5, color='red')

    plt.pause(0.1)

sim = Simulator(
    environment,
    [
        RobotControl(robot, controller),
    ],
    [
        SensorState(gps, robot),
        SensorState(odom, robot),
        SensorState(field_sensor, InsituInstrumentState(robot, value_field)),
        SensorState(field_sensor_locations, robot),
    ],
    DT,
    renderer=plot
)

# Run the simulator
results = sim.run()

plt.show()