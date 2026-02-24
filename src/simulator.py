from typing import TypeVar

from environment import Environment
from robots import Robot
from sensors import Sensor
from utils import Pose

RobotState = TypeVar("RobotState", bound=Pose)
RobotControl = TypeVar("RobotControl", bound=object)
SensorState = TypeVar("SensorState", bound=object) 
SensorMeasurement = TypeVar("SensorMeasurement", bound=object)

class Simulator:
    """
    A class to orchestrate simulations   
    """
    def __init__(self,
        environment: Environment,
        robots_with_control: dict[Robot[RobotState, RobotControl], list[RobotControl]],
        sensors_with_state: dict[Sensor[SensorState, SensorMeasurement], SensorState],
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
        self.robots_with_control = robots_with_control
        self.sensors_with_state = sensors_with_state
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
        for robot, control in self.robots_with_control.items():
            new_state = robot.step(control[self.i], self.dt)

            # Don't update the robots position if it collides with something
            # TODO: Make this slide or go partway instead of just stopping
            if self.valid_pose(new_state):
                robot.state = new_state

        # update the time
        self.i += 1

    def measure(self) -> dict[Sensor[object, SensorMeasurement], SensorMeasurement]:
        """
        Take measurements from all sensors

        Returns:
            a dictionary where the keys are the sensors that took measurements, and the values are the measurements
        """
        measurements = {}

        # Take measurements
        for sensor, state, in self.sensors_with_state.items():
            measurement = sensor.measure(state, self.time)
            if measurement is not None:
                measurements[sensor] = measurement
        
        return measurements

    def run(self) -> dict[Sensor[object, SensorMeasurement], list[SensorMeasurement]]:
        """
        Run the simulation
        
        Returns:
            a dictionary where the keys are the sensors in the simulation, and the values are the lists of measurements over time
        """
        steps: float = min([len(control) for control in self.robots_with_control.values()])

        # This assumes that all sensors return on the first measurement, which should 
        # be true because the last_measurement field defaults to -inf
        results: dict[Sensor[object, SensorMeasurement], list[SensorMeasurement]] = {}
        for sensor, measurement in self.measure().items():
            results[sensor] = [measurement]

        for _ in range(steps):
            self.step()
            for sensor, measurement in self.measure().items():
                results[sensor].append(measurement)
            
        return results