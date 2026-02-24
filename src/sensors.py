from abc import ABC, abstractmethod
from math import sqrt
from random import gauss
from typing import Generic, TypeVar

from robots import Robot
from utils import Pose, Vector

State = TypeVar("State", bound=object) 
Measurement = TypeVar("Measurement", bound=object)

class Sensor(ABC, Generic[State, Measurement]):
    """
    An abstract class for robots
    """

    def __init__(self, period: float) -> None:
        super().__init__()
        self.robot = Robot
        self.period = period
        self.last_measurement: float = float('-inf')

    @abstractmethod
    def ground_truth(self, state: State) -> Measurement:
        pass
    
    @abstractmethod
    def noisify(self, measurement: Measurement) -> Measurement:
        pass

    def measure(self, state: State, time: float) -> Measurement | None:
        if time < self.last_measurement + self.period:
            return None
        self.last_measurement = time
        return self.noisify(self.ground_truth(state))


class GPS(Sensor[Robot, Pose]):
    def __init__(self, period: float = 0.0, variance: Pose = Pose()) -> None:
        super().__init__(period)
        self.variance = variance
    
    def ground_truth(self, state: Robot) -> Pose:
        return state.state
    
    def noisify(self, measurement: Pose) -> Pose:
        return Pose(
            Vector(
                gauss(measurement.pos.x, sqrt(self.variance.pos.x)),
                gauss(measurement.pos.y, sqrt(self.variance.pos.y))
            ),
            gauss(measurement.theta, sqrt(self.variance.theta))
        )
