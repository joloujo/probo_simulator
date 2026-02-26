from dataclasses import dataclass
from typing import Any, Collection, Generic, Sequence, TypeVar 

from environment import Environment
from robots import Robot
from sensors import Sensor
from utils import Pose


# Results class modified from ChatGPT :)

RESULTS_SENSOR = TypeVar("RESULTS_SENSOR")
RESULTS_MEASUREMENT = TypeVar("RESULTS_MEASUREMENT")

class Results:
    def __init__(self) -> None:
        self._data: dict[Sensor[Any, Any], list[Any]] = {}

    def __getitem__(self, sensor: Sensor[RESULTS_SENSOR, RESULTS_MEASUREMENT]) -> list[RESULTS_MEASUREMENT]:
        # The cast is safe because _append preserved the pairing
        from typing import cast
        return cast(list[RESULTS_MEASUREMENT], self._data.get(sensor, []))

    def append(self, sensor: Sensor[RESULTS_SENSOR, RESULTS_MEASUREMENT], measurement: RESULTS_MEASUREMENT) -> None:
        if sensor not in self._data:
            self._data[sensor] = []
        self._data[sensor].append(measurement)


RC_STATE = TypeVar("RC_STATE", bound=Pose)
RC_CONTROL = TypeVar("RC_CONTROL")

@dataclass
class RobotControl(Generic[RC_STATE, RC_CONTROL]):
    robot: Robot[RC_STATE, RC_CONTROL]
    controls: Sequence[RC_CONTROL]

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
        """
        # Update state
        for binding in self.robots:
            if len(binding.controls) > self.i:
                new_state = binding.robot.step(binding.controls[self.i], self.dt)

                # Don't update the robots position if it collides with something
                # TODO: Make this slide or go partway instead of just stopping
                if self.valid_pose(new_state):
                    binding.robot.state = new_state

        # update the time
        self.i += 1

    def measure(self, results: Results):
        """
        Take measurements from all sensors

        Returns:
            a dictionary where the keys are the sensors that took measurements, and the values are the measurements
        """
        measurements = {}

        # Take measurements
        for binding in self.sensors:
            measurement = binding.sensor.measure(binding.state, self.time)
            if measurement is not None:
                results.append(binding.sensor, measurement)
        
        return measurements

    def run(self) -> Results:
        """
        Run the simulation
        
        Returns:
            a dictionary where the keys are the sensors in the simulation, and the values are the lists of measurements over time
        """
        steps = min([len(binding.controls) for binding in self.robots])
        results = Results()

        for _ in range(steps):
            self.step()
            self.measure(results)
            
        return results
