from itertools import product
import matplotlib.pyplot as plt
import numpy as np
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF

from probo_sim.environment import Environment
from probo_sim.robots import DifferentialDrive, DifferentialDriveState, DifferentialDriveControl
from probo_sim.sensors import GPS, InsituInstrument, InsituInstrumentState
from probo_sim.simulator import Simulator, RobotControl, SensorState, open_loop
from probo_sim.utils import Bounds, Vector, gaussian
from probo_sim.visualizer import Visualizer

DT = 0.1
SENSOR_PERIOD = 1
VARIANCE = 0.01
LENGTH_SCALE = 2.5

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
        Bounds(Vector(0, 2), Vector(4, 4)),
        Bounds(Vector(2, 6), Vector(4, 8)),
        Bounds(Vector(6, 0), Vector(8, 4)),
    ],
)

robot_start = DifferentialDriveState(
    Vector(1, 1),
)
robot = DifferentialDrive(robot_start)
control: list[DifferentialDriveControl] = [
    DifferentialDriveControl(1, 0) ] * 30 + [ # Go forward three units
    DifferentialDriveControl(1, 1)] * 16 + [ # Turn left ~90 degrees
    DifferentialDriveControl(1, 0)] * 20 + [ # Go forward two units
    DifferentialDriveControl(1, -1)] * 16 + [ # Turn right ~90 degrees
    DifferentialDriveControl(1, 0)] * 8 + [ # Go forward a bit more
    DifferentialDriveControl(1, 1/2)] * 94 + [ # Go forward a bit more
    DifferentialDriveControl(1, -1/2)] * 31 # Go forward a bit more

gps = GPS()
field_sensor_locations = GPS(SENSOR_PERIOD)
gt_field_sensor = InsituInstrument(SENSOR_PERIOD)
field_sensor = InsituInstrument(SENSOR_PERIOD, VARIANCE)

sim = Simulator(
    environment,
    [
        RobotControl(robot, open_loop(control)),
    ],
    [
        SensorState(gps, robot),
        SensorState(field_sensor_locations, robot),
        SensorState(gt_field_sensor, InsituInstrumentState(robot, value_field)),
        SensorState(field_sensor, InsituInstrumentState(robot, value_field)),
    ],
    DT
)

# Run the simulator
results = sim.run()

# Fit the field
# belief = GaussianProcessRegressor(
#     kernel = 1 * RBF([1, 1], (0.01, 100)),
#     alpha = VARIANCE,
#     n_restarts_optimizer=20,
# )

belief = GaussianProcessRegressor(
    # kernel = 1.0 * RBF([LENGTH_SCALE, LENGTH_SCALE]),
    kernel = RBF([LENGTH_SCALE, LENGTH_SCALE], 'fixed'),
    # n_restarts_optimizer=15,
)

belief.fit(
    np.asarray([(pose.pos.x, pose.pos.y) for pose in results[field_sensor_locations]]), 
    np.asarray(results[field_sensor])
)

# print(f'This is how many: {len(results[field_sensor])}')

print(belief.kernel_)

x = np.linspace(environment.bounds.min.x, environment.bounds.max.x, 21)
y = np.linspace(environment.bounds.min.y, environment.bounds.max.y, 21)
X, Y = np.meshgrid(x, y)
M = np.array(list(product(x,y)))

c_sample, std_dev = belief.predict(M, return_std=True) # type: ignore

# print(c_sample)

# print(std_dev)

print(f'std of noise is: {np.std([gt - noisy for gt, noisy in zip(results[gt_field_sensor], results[field_sensor])])}')

# Visualize the results
fig, ax = plt.subplots(1, 2)

Visualizer.plot_environment(ax[0], environment)
Visualizer.plot_field_values(ax[0], x, y, c_sample)

# viz.plot_field(value_field, environment.bounds, count=(21, 21), levels=20)
Visualizer.plot_poses(ax[0], [robot_start] + results[gps], alpha=0.5, color='red')

Visualizer.plot_environment(ax[1], environment)
Visualizer.plot_field_values(ax[1], x, y, std_dev, levels=50)
Visualizer.plot_poses(ax[1], results[field_sensor_locations], alpha=0.5, color='red')

plt.show()
