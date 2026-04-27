from itertools import product
import matplotlib.pyplot as plt
from math import cos, sqrt, pi
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
B = 2
DISTANCE_MULTIPLIER = 0.02

E_D = 2.5
PLANNING_ROLLOUTS = 500
HORIZON = 50

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
    for (v, w) in product(np.linspace(0., 1., 3), np.linspace(-3, 3, 13))
]

# Calculate belief
belief = GaussianProcessRegressor(
    kernel = RBF([LENGTH_SCALE, LENGTH_SCALE], 'fixed'),
)

# Visualize the results
fig, ax = plt.subplots(2, 2)
manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize()) # type: ignore

goal = Vector()

def controller_2_electric_boogaloo(sim: Simulator) -> DifferentialDriveControl | None:
    global goal

    if sim.i == 0:
        return DifferentialDriveControl(1, 0)
    
    if not plt.fignum_exists(fig.number):
        return None

    current_pose = sim.results[gps][-1]

    belief.fit(
        np.asarray([(pose.pos.x, pose.pos.y) for pose in sim.results[field_sensor_locations]]), 
        np.asarray(sim.results[field_sensor])
    )

    x = np.linspace(environment.bounds.min.x, environment.bounds.max.x, 41)
    y = np.linspace(environment.bounds.min.y, environment.bounds.max.y, 41)
    M = np.array(list(product(x,y)))
    c_sample, std_dev = belief.predict(M, return_std=True) # type: ignore

    distance = np.linalg.norm(M - np.array([[current_pose.pos.x, current_pose.pos.y]]), axis=1)

    print(distance)

    reward = c_sample + sqrt(B) * std_dev - distance * DISTANCE_MULTIPLIER

    goal_tuple = M[np.argmax(reward)]
    goal = Vector(goal_tuple[0], goal_tuple[1])

    diff = goal - current_pose.pos

    if diff.r < 0.1: return None

    angle = (diff.theta - current_pose.theta + pi) % (2 * pi) - pi
    v = min(cos(angle), diff.r)
    w = max(-1, min(angle, 1))

    return DifferentialDriveControl(v, w)


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
    # total_reward_accumulated: list[float] = [0.] * len(action_library)

    # for i, action in enumerate(action_library):
    #     next_state = DifferentialDrive.kinematics(
    #         DifferentialDriveState(current_pose.pos, current_pose.theta), 
    #         action,
    #         DT,
    #     )

    #     for _ in range(20):
    #         state = [next_state]

    #         for _ in range(HORIZON - 1):
    #             state.append(DifferentialDrive.kinematics(
    #                 DifferentialDriveState(state[-1].pos, state[-1].theta), 
    #                 random.choice(action_library),
    #                 DT,
    #             ))
            
    #         mean, stddev = belief.predict([(s.pos.x, s.pos.y) for s in state], return_std=True) # type: ignore

    #         reward = sum(mean + np.sqrt(B) * stddev)

    #         total_reward_accumulated[i] += reward

    action_next_state = [
        DifferentialDrive.kinematics(
            DifferentialDriveState(current_pose.pos, current_pose.theta), 
            action,
            DT,
        )
        for action in action_library
    ]
    total_reward_accumulated = np.zeros((len(action_library)))
    times_simulated = np.zeros((len(action_library)))

    for _ in range(PLANNING_ROLLOUTS):
        # Select action to test

        total_times_simulated = sum(times_simulated)
        average_reward_across_all = sum(total_reward_accumulated) / total_times_simulated

        Q_star = np.array([
            Q / N + sqrt(total_times_simulated ** E_D / N)
            if N != 0 
            else float('inf')
            for Q, N, in zip(total_reward_accumulated, times_simulated)
        ])

        selected_action_index = np.argmax(Q_star)

        state = [action_next_state[selected_action_index]]

        for _ in range(HORIZON - 1):

            state.append(DifferentialDrive.kinematics(
                DifferentialDriveState(state[-1].pos, state[-1].theta), 
                random.choice(action_library),
                DT,
            ))

            mean, stddev = belief.predict([(s.pos.x, s.pos.y) for s in state], return_std=True) # type: ignore

            reward = sum(mean + np.sqrt(B) * stddev)

            total_reward_accumulated[selected_action_index] += reward

        times_simulated[selected_action_index] += 1

    # print(times_simulated)
    
    average_reward = [Q / N for Q, N, in zip(total_reward_accumulated, times_simulated)]

    best_action = action_library[np.argmax(average_reward)]
    # best_action = action_library[np.argmax(total_reward_accumulated)]

    return best_action

ground_truth_plot: Axes = ax[0, 0]
belief_plot: Axes = ax[1, 0]
uncertainty_plot: Axes = ax[1, 1]
planning_plot: Axes = ax[0, 1]

def plot(sim: Simulator):
    for axis in ax.flatten():
        Visualizer.plot_environment(axis, environment)
        axis.clear()

    ground_truth_plot.set_title('Ground Truth')
    belief_plot.set_title('Belief')
    uncertainty_plot.set_title('Uncertainty')
    planning_plot.set_title('Reward')

    ground_truth_plot.text(0, 1.05, f'Time: {sim.time}', transform=ground_truth_plot.transAxes)

    x = np.linspace(environment.bounds.min.x, environment.bounds.max.x, 21)
    y = np.linspace(environment.bounds.min.y, environment.bounds.max.y, 21)
    M = np.array(list(product(x,y)))
    c_sample, std_dev = belief.predict(M, return_std=True) # type: ignore

    current_pose = sim.results[gps][-1]
    distance = np.linalg.norm(M - np.array([[current_pose.pos.x, current_pose.pos.y]]), axis=1)

    reward = c_sample + sqrt(B) * std_dev - distance * DISTANCE_MULTIPLIER

    Visualizer.plot_field(ground_truth_plot, value_field, environment.bounds, levels=np.linspace(0, 1.1, 23))
    Visualizer.plot_field_values(belief_plot, x, y, c_sample, levels=np.linspace(0, 1.1, 23))
    Visualizer.plot_field_values(uncertainty_plot, x, y, std_dev, levels=20)
    Visualizer.plot_field_values(planning_plot, x, y, reward, levels=20)

    Visualizer.plot_poses(ground_truth_plot, [robot_start] + sim.results[gps], alpha=0.5, color='red')
    Visualizer.plot_vector(planning_plot, goal, marker='x', ms=3, mec='red')

    plt.pause(0.1)

sim = Simulator(
    environment,
    [
        RobotControl(robot, controller_2_electric_boogaloo),
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