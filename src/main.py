from environment import Environment
from robots import DifferentialDrive, DifferentialDriveState, DifferentialDriveControl, HolonomicDrive, HolonomicDriveState, HolonomicDriveControl
from sensors import GPS, IMU, Encoder
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

# robot = DifferentialDrive(
#     DifferentialDriveState(
#         Vector(1, 1)
#     )
# )
# control: list[DifferentialDriveControl] = [
#     DifferentialDriveControl(1, 0) ] * 30 + [ # Go forward three units
#     DifferentialDriveControl(1, 1)] * 16 + [ # Turn left ~90 degrees
#     DifferentialDriveControl(1, 0)] * 20 + [ # Go forward two units
#     DifferentialDriveControl(1, -1)] * 16 # Turn right ~90 degrees

# odom = Encoder(0)

robot = HolonomicDrive(
    HolonomicDriveState(
        Vector(1, 1)
    )
)
control: list[HolonomicDriveControl] = [
    HolonomicDriveControl(1, 0, 0)] * 40 + [ # Go right four units
    HolonomicDriveControl(0, 1, 31.415/20)] * 40 + [ # Go up four units
    HolonomicDriveControl(-1, 0, 0)] * 20 # Go left two units

odom = IMU(0)

gps = GPS()

sim = Simulator(
    environment,
    {robot: control},
    {
        gps: robot, # type: ignore
        odom: robot,
    },
    DT
)

# Run the simulator
results = sim.run()

# p = [DifferentialDriveState(Vector(1, 1))]
# for measurement in results[odom]: # type: ignore
#     p.append(DifferentialDrive.kinematics(p[-1], DifferentialDriveControl(
#         measurement.v,
#         measurement.w
#     ), DT))

p = [HolonomicDriveState(Vector(1, 1))]
for measurement in results[odom]: # type: ignore
    p.append(HolonomicDrive.kinematics(p[-1], HolonomicDriveControl(
        measurement.xv,
        measurement.yv,
        measurement.w
    ), DT))

# Visualize the results
viz = Visualizer()
viz.plot_environment(environment)
viz.plot_poses(results[gps], alpha=0.5, color='blue') # type: ignore
viz.plot_poses(p, alpha=0.5, color='red') # type: ignore
viz.show()
