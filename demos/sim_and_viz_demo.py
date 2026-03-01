from probo_sim.environment import Environment
from probo_sim.robots import DifferentialDrive, DifferentialDriveState, DifferentialDriveControl, HolonomicDrive, HolonomicDriveState, HolonomicDriveControl
from probo_sim.sensors import GPS, IMU, Encoder
from probo_sim.simulator import Simulator, RobotControl, SensorState
from probo_sim.utils import Bounds, Vector
from probo_sim.visualizer import Visualizer

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
    Vector(1, 1.1)
)
diff_robot = DifferentialDrive(diff_start)
diff_control: list[DifferentialDriveControl] = [
    DifferentialDriveControl(1, 0) ] * 30 + [ # Go forward three units
    DifferentialDriveControl(1, 1)] * 16 + [ # Turn left ~90 degrees
    DifferentialDriveControl(1, 0)] * 20 + [ # Go forward two units
    DifferentialDriveControl(1, -1)] * 16 + [# Turn right ~90 degrees
    DifferentialDriveControl(1, 0)] * 18 # Go forward a bit more

diff_gps = GPS()
diff_odom = Encoder(0)

holo_start = HolonomicDriveState(
    Vector(1, 1)
)
holo_robot = HolonomicDrive(holo_start)
holo_control: list[HolonomicDriveControl] = [
    HolonomicDriveControl(1, 0, 0)] * 40 + [ # Go right four units
    HolonomicDriveControl(0, 1, 31.415/20)] * 40 + [ # Go up four units
    HolonomicDriveControl(-1, 0, 0)] * 20 # Go left two units

holo_gps = GPS()
holo_odom = IMU(0)

sim = Simulator(
    environment,
    [
        RobotControl(diff_robot, diff_control),
        RobotControl(holo_robot, holo_control),
    ],
    [
        SensorState(diff_gps, diff_robot),
        SensorState(diff_odom, diff_robot),
        SensorState(holo_gps, holo_robot),
        SensorState(holo_odom, holo_robot),
    ],
    DT
)

# Run the simulator
results = sim.run()

diff_calculated_state = [diff_start]
for measurement in results[diff_odom]:
    diff_calculated_state.append(DifferentialDrive.kinematics(diff_calculated_state[-1], DifferentialDriveControl(
        measurement.v,
        measurement.w
    ), DT))

holo_calculated_state = [holo_start]
for measurement in results[holo_odom]:
    holo_calculated_state.append(HolonomicDrive.kinematics(holo_calculated_state[-1], HolonomicDriveControl(
        measurement.xv,
        measurement.yv,
        measurement.w
    ), DT))

# Visualize the results
viz = Visualizer()
viz.plot_environment(environment)

viz.plot_poses([diff_start] + results[diff_gps], alpha=0.5, color='red')
viz.plot_poses(diff_calculated_state, alpha=0.5, color='orange')

viz.plot_poses([holo_start] + results[holo_gps], alpha=0.5, color='blue')
viz.plot_poses(holo_calculated_state, alpha=0.5, color='green')

viz.show()
