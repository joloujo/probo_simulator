from dataclasses import dataclass
from typing import Any, Callable, Collection, Generic, Sequence, TypeVar

from probo_sim.environment import Environment
from probo_sim.robots import Robot
from probo_sim.sensors import Sensor
from probo_sim.utils import Pose


# Results class modified from ChatGPT :)

RESULTS_INPUT_SENSOR = TypeVar("RESULTS_INPUT_SENSOR")
RESULTS_INPUT_ROBOT = TypeVar("RESULTS_INPUT_ROBOT", bound=Pose)
RESULTS_OUTPUT = TypeVar("RESULTS_OUTPUT")

class Results:
    def __init__(self) -> None:
        self._data: dict[Sensor[Any, Any] | Robot[Any, Any], list[Any]] = {}

    def __getitem__(self, input: Sensor[RESULTS_INPUT_SENSOR, RESULTS_OUTPUT] | Robot[RESULTS_INPUT_ROBOT, RESULTS_OUTPUT]) -> list[RESULTS_OUTPUT]:
        # The cast is safe because _append preserved the pairing
        from typing import cast
        return cast(list[RESULTS_OUTPUT], self._data.get(input, []))

    def append(self, input: Sensor[RESULTS_INPUT_SENSOR, RESULTS_OUTPUT] | Robot[RESULTS_INPUT_ROBOT, RESULTS_OUTPUT], output: RESULTS_OUTPUT) -> None:
        if input not in self._data:
            self._data[input] = []
        self._data[input].append(output)


RC_STATE = TypeVar("RC_STATE", bound=Pose)
RC_CONTROL = TypeVar("RC_CONTROL")

@dataclass
class RobotControl(Generic[RC_STATE, RC_CONTROL]):
    robot: Robot[RC_STATE, RC_CONTROL]
    controller: Callable[['Simulator'], RC_CONTROL | None]

def open_loop(actions: Sequence[RC_CONTROL]) -> Callable[['Simulator'], RC_CONTROL | None]:
    def controller(sim: 'Simulator') -> RC_CONTROL | None:
        i = sim.i
        return actions[i] if i < len(actions) else None
    return controller

SS_STATE = TypeVar("SS_STATE")
SS_MEASUREMENT = TypeVar("SS_MEASUREMENT")

@dataclass
class SensorState(Generic[SS_STATE, SS_MEASUREMENT]):
    sensor: Sensor[SS_STATE, SS_MEASUREMENT]
    state: SS_STATE

class Simulator:
    """
    A class to orchestrate simulations   
    """
    def __init__(self,
        environment: Environment,
        robots: Collection[RobotControl],
        sensors: Collection[SensorState],
        dt: float,
        renderer: Callable[['Simulator'], Any] | None = None
    ) -> None:
        """
        Create the simulator

        Params:
            environment: The environment that the simulation takes place in
            robots_with_control: A dictionary where the keys are the robots in the simulation and the values are the control commands for each timestep
            sensors_with_state: A dictionatry where the keys are the sensors in the simulation and the values are the context they need for measurements
            dt: the length of one simulation timestep
        """
        self.environment = environment
        self.robots = robots
        self.sensors = sensors
        self.dt = dt
        self.i = 0
        self.results = Results()
        self.renderer = renderer
    
    @property
    def time(self) -> float:
        """
        The current simulation time

        Returns:
            the current simulation time
        """
        return self.i * self.dt
    
    def valid_pose(self, pose: Pose) -> bool:
        """
        Check to see if a pose is valid in the environment

        Params:
            pose: the pose to check
        
        Returns:
            whether or not the pose is valid
        """
        # Make sure the pose is in bounds
        if not self.environment.bounds.contains(pose.pos):
                return False
        
        # Make sure the pose isn't in any obstacles
        for obstacle in self.environment.obstacles:
            if obstacle.contains(pose.pos):
                return False
        
        return True

    def step(self):
        """
        Execute one timestep in the simulation

        Returns:
            (bool) true if the simulation should continue running
        """

        any_active = False

        # Update state
        for binding in self.robots:

            control = binding.controller(self)

            active = control is not None
            any_active = any_active or active

            if active:
                self.results.append(binding.robot, control)

                new_state = binding.robot.step(control, self.dt)

                # Don't update the robots position if it collides with something
                # TODO: Make this slide or go partway instead of just stopping
                if self.valid_pose(new_state):
                    binding.robot.state = new_state

        # update the time
        self.i += 1

        return active

    def measure(self):
        """
        Take measurements from all sensors

        Returns:
            a dictionary where the keys are the sensors that took measurements, and the values are the measurements
        """
        # Take measurements
        for binding in self.sensors:
            measurement = binding.sensor.measure(binding.state, self.time)
            if measurement is not None:
                self.results.append(binding.sensor, measurement)
        
    def run(self) -> Results:
        """
        Run the simulation
        
        Returns:
            a dictionary where the keys are the sensors in the simulation, and the values are the lists of measurements over time
        """
        self.results = Results()

        while True:
            any_active = self.step()

            if not any_active:
                break

            self.measure()

            if self.renderer is not None:
                self.renderer(self)
            
        return self.results
