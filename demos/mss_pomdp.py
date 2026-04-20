from itertools import product
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
B = 5

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
    for (v, w) in product([-1.0, -0.5, 0.0, 0.5, 1.0], [-1.0, -0.5, 0.0, 0.5, 1.0])
]

def controller(sim: Simulator) -> DifferentialDriveControl | None:

    if sim.i == 0:
        return DifferentialDriveControl(1, 0)
    
    if sim.i == 200:
        return None

    # Calculate belief
    belief = GaussianProcessRegressor(
        kernel = RBF([LENGTH_SCALE, LENGTH_SCALE], 'fixed'),
    )

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

        for _ in range(20):

            next_pose = DifferentialDrive.kinematics(
                DifferentialDriveState(current_pose.pos, current_pose.theta), 
                action,
                DT,
            )
            
            for _ in range(50):
                next_pose = DifferentialDrive.kinematics(
                    DifferentialDriveState(next_pose.pos, next_pose.theta), 
                    random.choice(action_library),
                    DT,
                )
            
            mean, stddev = belief.predict([(next_pose.pos.x, next_pose.pos.x)], return_std=True) # type: ignore

            reward = float((mean + B * stddev)[0])

            total_reward += reward
            best_reward = max(reward, best_reward)
        
        best_rewards.append(best_reward)
        total_rewards.append(total_reward)

    best_action = action_library[np.argmax(best_rewards)]
    # best_action = action_library[np.argmax(total_rewards)]

    return best_action

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
    DT
)

# Run the simulator
results = sim.run()


# Visualize the results
viz = Visualizer()
viz.plot_environment(environment)

viz.plot_field(value_field, environment.bounds, levels=20)
viz.plot_poses([robot_start] + results[gps], alpha=0.5, color='red')

viz.show()
