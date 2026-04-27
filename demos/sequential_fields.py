from enum import Enum, auto
from itertools import product
import matplotlib.pyplot as plt
from math import cos, sqrt, pi
from matplotlib.axes import Axes
import numpy as np
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF

from probo_sim.environment import Environment
from probo_sim.robots import DifferentialDrive, DifferentialDriveState, DifferentialDriveControl
from probo_sim.sensors import GPS, Encoder, InsituInstrument, InsituInstrumentState
from probo_sim.simulator import Simulator, RobotControl, SensorState, open_loop
from probo_sim.utils import Bounds, Vector, gaussian
from probo_sim.visualizer import Visualizer

# Set up the states for the state machine
class State(Enum):
    FIELD_1 = auto()
    FIELD_2 = auto()
    DONE = auto()

# Set the task order
task_order: list[State] = [State.FIELD_1, State.FIELD_2, State.DONE]

# Set up constants
DT = 0.5 # Timestep length
SENSOR_PERIOD = 1 # How often the field sensors read
VARIANCE = 0 # The variance of the field sensors
B = 2 # The multiplier for the uncertainty in the reward function
DISTANCE_MULTIPLIER = 0.02 # The multiplier for the distance in the reward function

# The first field, with longer length scale
def value_field_1(position: Vector) -> float:

    point = np.array([position.x, position.y])

    return sum([
        gaussian(point, 0.8, np.array([2, 8]), np.array([4])),
        gaussian(point, 0.4, np.array([5, 3]), np.array([6, 4])),
    ])

# The second field, with shorter length scale
def value_field_2(position: Vector) -> float:

    point = np.array([position.x, position.y])

    return sum([
        gaussian(point, 1, np.array([8, 8]), np.array([2])),
        gaussian(point, 0.6, np.array([2, 5]), np.array([4, 2])),
        gaussian(point, 0.8, np.array([9, 1]), np.array([4]))
    ])


# Set up the simulator
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
# Create sensors for both fields and a gps to get the location whenever the values are measured
field_sensor_1 = InsituInstrument(SENSOR_PERIOD, VARIANCE)
field_sensor_2 = InsituInstrument(SENSOR_PERIOD, VARIANCE)
field_sensor_locations = GPS(SENSOR_PERIOD)

# Be ready to calculate belief
belief_1 = GaussianProcessRegressor(
    kernel = RBF([4, 4], 'fixed'),
)

belief_2 = GaussianProcessRegressor(
    kernel = RBF([1, 1], 'fixed'),
)

# Be ready to visualize the results
fig, ax = plt.subplots(2, 4)
# Set the view to fullscreen
manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize()) # type: ignore

# Make the goal and reward global so the the controller and visualizer can both use it
goal = Vector()
reward = None

# Start at task 0
task_number = 0

def controller(sim: Simulator) -> DifferentialDriveControl | None:
    global goal, task_number, reward     

    # Without measurements, just go forward a bit
    if sim.i == 0:
        return DifferentialDriveControl(1, 0)
    
    # If the visualizer is closed, stop
    if not plt.fignum_exists(fig.number):
        return None

    # Get the robots current pose
    current_pose = sim.results[gps][-1]

    # Update belief and reward based on the current task
    if task_order[task_number] == State.FIELD_1:
        belief_1.fit(
            np.asarray([(pose.pos.x, pose.pos.y) for pose in sim.results[field_sensor_locations]]), 
            np.asarray(sim.results[field_sensor_1])
        )

        x = np.linspace(environment.bounds.min.x, environment.bounds.max.x, 41)
        y = np.linspace(environment.bounds.min.y, environment.bounds.max.y, 41)
        M = np.array(list(product(x,y)))
        c_sample_1, std_dev_1 = belief_1.predict(M, return_std=True) # type: ignore
        c_sample_2, std_dev_2 = belief_2.predict(M, return_std=True) # type: ignore

        distance = np.linalg.norm(M - np.array([[current_pose.pos.x, current_pose.pos.y]]), axis=1)

        reward = \
            c_sample_1 + sqrt(B) * std_dev_1 + \
            -distance * DISTANCE_MULTIPLIER
        
        goal_tuple = M[np.argmax(reward)]
        goal = Vector(goal_tuple[0], goal_tuple[1])   

    if task_order[task_number] == State.FIELD_2:
        belief_2.fit(
            np.asarray([(pose.pos.x, pose.pos.y) for pose in sim.results[field_sensor_locations]]), 
            np.asarray(sim.results[field_sensor_2])
        )

        x = np.linspace(environment.bounds.min.x, environment.bounds.max.x, 41)
        y = np.linspace(environment.bounds.min.y, environment.bounds.max.y, 41)
        M = np.array(list(product(x,y)))
        c_sample_1, std_dev_1 = belief_1.predict(M, return_std=True) # type: ignore
        c_sample_2, std_dev_2 = belief_2.predict(M, return_std=True) # type: ignore

        distance = np.linalg.norm(M - np.array([[current_pose.pos.x, current_pose.pos.y]]), axis=1)

        reward = \
            c_sample_2 + sqrt(B) * std_dev_2 + \
            -distance * DISTANCE_MULTIPLIER

        goal_tuple = M[np.argmax(reward)]
        goal = Vector(goal_tuple[0], goal_tuple[1])   

    # Once the goal is determined for the current task, try to go towards it
    diff = goal - current_pose.pos

    if diff.r < 0.1:
        task_number += 1
    
    if task_order[task_number] == State.DONE:
        return None

    angle = (diff.theta - current_pose.theta + pi) % (2 * pi) - pi
    v = min(cos(angle), diff.r)
    w = max(-1, min(angle, 1))

    # Make sure the robot will actually move successfully 
    next_pose = DifferentialDrive.kinematics(
        DifferentialDriveState(
            current_pose.pos, 
            current_pose.theta
        ),
        DifferentialDriveControl(v, w),
        DT
    )
    if sim.valid_pose(next_pose):
        return DifferentialDriveControl(v, w)
    else:
        # If the robot will go out of bounds, then just turn
        return DifferentialDriveControl(0, w)

# Label the plots so the code is clearer
ground_truth_plot_1: Axes = ax[0, 0]
belief_plot_1: Axes = ax[0, 1]
uncertainty_plot_1: Axes = ax[0, 2]
ground_truth_plot_2: Axes = ax[1, 0]
belief_plot_2: Axes = ax[1, 1]
uncertainty_plot_2: Axes = ax[1, 2]
planning_plot: Axes = ax[0, 3]

# Visualise the state of the simulator
def plot(sim: Simulator):
    # Clear the plots
    for axis in ax.flatten():
        Visualizer.plot_environment(axis, environment)
        axis.clear()

    # Label the plots
    ground_truth_plot_1.set_title('Ground Truth: Field 1')
    belief_plot_1.set_title('Belief: Field 1')
    uncertainty_plot_1.set_title('Uncertainty: Field 1')
    ground_truth_plot_2.set_title('Ground Truth: Field 2')
    belief_plot_2.set_title('Belief: Field 2')
    uncertainty_plot_2.set_title('Uncertainty: Field 2')
    planning_plot.set_title('Reward')

    # List the current time
    planning_plot.text(0, -0.1, f'Time: {sim.time}', transform=planning_plot.transAxes)

    # Recalculate the reward field to visualize it (sad code :( )
    x = np.linspace(environment.bounds.min.x, environment.bounds.max.x, 41)
    y = np.linspace(environment.bounds.min.y, environment.bounds.max.y, 41)
    M = np.array(list(product(x,y)))
    c_sample_1, std_dev_1 = belief_1.predict(M, return_std=True) # type: ignore
    c_sample_2, std_dev_2 = belief_2.predict(M, return_std=True) # type: ignore

    # Acutally plot everything
    Visualizer.plot_field(ground_truth_plot_1, value_field_1, environment.bounds, levels=np.linspace(0, 1.1, 23))
    Visualizer.plot_field_values(belief_plot_1, x, y, c_sample_1, levels=np.linspace(0, 1.1, 23))
    Visualizer.plot_field_values(uncertainty_plot_1, x, y, std_dev_1, levels=20)

    Visualizer.plot_field(ground_truth_plot_2, value_field_2, environment.bounds, levels=np.linspace(0, 1.1, 23))
    Visualizer.plot_field_values(belief_plot_2, x, y, c_sample_2, levels=np.linspace(0, 1.1, 23))
    Visualizer.plot_field_values(uncertainty_plot_2, x, y, std_dev_2, levels=20)

    if reward is not None:
        Visualizer.plot_field_values(planning_plot, x, y, reward, levels=20)

    Visualizer.plot_poses(ground_truth_plot_1, [robot_start] + sim.results[gps], alpha=0.5, color='red')
    Visualizer.plot_poses(ground_truth_plot_2, [robot_start] + sim.results[gps], alpha=0.5, color='red')
    Visualizer.plot_vector(planning_plot, goal, marker='x', ms=3, mec='red')

    # Non-blocking real-time plot update
    plt.pause(0.1)

sim = Simulator(
    environment,
    [
        RobotControl(robot, controller),
    ],
    [
        SensorState(gps, robot),
        SensorState(odom, robot),
        SensorState(field_sensor_1, InsituInstrumentState(robot, value_field_1)),
        SensorState(field_sensor_2, InsituInstrumentState(robot, value_field_2)),
        SensorState(field_sensor_locations, robot),
    ],
    DT,
    renderer=plot
)

# Run the simulator
results = sim.run()

# Keep the plot open at the end to see the results
plt.show()