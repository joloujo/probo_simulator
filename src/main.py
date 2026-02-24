from environment import Environment
from robots import DifferentialDrive, DifferentialDriveState, DifferentialDriveControl
from sensors import GPS
from simulator import Simulator
from utils import Bounds, Vector
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
        Vector(2, 1)
    )
)
gps = GPS()

control: list[DifferentialDriveControl] = [
    DifferentialDriveControl(1, 0)
] * 20 + [
    DifferentialDriveControl(1, 1)
] * 16 + [
    DifferentialDriveControl(1, 0)
] * 20 + [
    DifferentialDriveControl(1, -1)
] * 16

sim = Simulator(
    environment,
    {robot: control},
    {gps: robot},
    DT
)

results = sim.run()

viz = Visualizer()
viz.plot_environment(environment)
viz.plot_poses(results[gps], alpha=0.5, color='blue') # type: ignore
viz.show()
