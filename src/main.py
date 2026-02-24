from environment import Environment
from robots import DifferentialDrive, DifferentialDriveState, DifferentialDriveControl
from sensors import GPS
from simulator import Simulator
from utils import Bounds, Vector, Pose
from visualizer import Visualizer

DT = 0.1

environment = Environment(
    Bounds(Vector(0, 0), Vector(10, 10)),
    [
        Bounds(Vector(0, 2), Vector(4, 4)),
        Bounds(Vector(2, 6), Vector(4, 8)),
        Bounds(Vector(6, 0), Vector(8, 4)),
    ],
)

robot = DifferentialDrive(
    DifferentialDriveState(
        Vector(1, 1)
    )
)
control: list[DifferentialDriveControl] = [
    DifferentialDriveControl(1, 0) ] * 30 + [ # Go forward three units
    DifferentialDriveControl(1, 1)] * 16 + [ # Turn left ~90 degrees
    DifferentialDriveControl(1, 0)] * 20 + [ # Go forward two units
    DifferentialDriveControl(1, -1)] * 16 # Turn right ~90 degrees

gps = GPS()

sim = Simulator(
    environment,
    {robot: control},
    {gps: robot},
    DT
)

# Run the simulator
results = sim.run()

# Visualize the results
viz = Visualizer()
viz.plot_environment(environment)
viz.plot_poses(results[gps], alpha=0.5, color='blue') # type: ignore
viz.show()
