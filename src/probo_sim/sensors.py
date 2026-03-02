from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from math import sqrt
from random import gauss
from typing import Generic, Sequence, TypeVar

from probo_sim.robots import Robot, DifferentialDrive, HolonomicDrive
from probo_sim.utils import Pose, Vector

State = TypeVar("State") 
Measurement = TypeVar("Measurement")

class Sensor(ABC, Generic[State, Measurement]):
    """
    An abstract class for sensors
    """

    def __init__(self, period: float) -> None:
        super().__init__()
        self.period = period
        self.last_measurement: float = float('-inf')

    @abstractmethod
    def ground_truth(self, state: State) -> Measurement:
        """
        Make a ground truth measurement

        Params:
            state: The state to base the measurement on

        Returns:
            the ground truth measurement with no noise
        """
    
    @abstractmethod
    def noisify(self, measurement: Measurement) -> Measurement:
        """
        Add noise to the measurement

        Params:
            measurement: The measurement to add noise to
        
        Returns:
            the measurement with noise
        """

    def measure(self, state: State, time: float) -> Measurement | None:
        """
        Make a noisy measurement if able. If a measurement was made, reset the time of the most recent measurement

        Params:
            state: The state to base the measurement on
            time: The current time

        Returns:
            the noisy measurement if able, otherwise None
        """
        if time < self.last_measurement + self.period:
            return None
        self.last_measurement = time
        return self.noisify(self.ground_truth(state))


class GPS(Sensor[Robot, Pose]):
    def __init__(self, period: float = 0.0, variance: Pose | None = None) -> None:
        super().__init__(period)
        self.variance = variance if variance is not None else Pose() 
    
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


@dataclass
class IMUMeasurement:
    xv: float = 0.0
    yv: float = 0.0
    w: float = 0.0

class IMU(Sensor[HolonomicDrive, IMUMeasurement]):
    def __init__(self, period: float, variance: IMUMeasurement | None = None) -> None:
        super().__init__(period)
        self.variance = variance if variance is not None else IMUMeasurement()

    def ground_truth(self, state: HolonomicDrive) -> IMUMeasurement:
        return IMUMeasurement(
            state.state.xv,
            state.state.yv,
            state.state.w,
        )
    
    def noisify(self, measurement: IMUMeasurement) -> IMUMeasurement:
        return IMUMeasurement(
            gauss(measurement.xv, sqrt(self.variance.xv)),
            gauss(measurement.yv, sqrt(self.variance.yv)),
            gauss(measurement.w, sqrt(self.variance.w))
        )


@dataclass
class EncoderMeasurement:
    v: float = 0.0
    w: float = 0.0

class Encoder(Sensor[DifferentialDrive, EncoderMeasurement]):
    def __init__(self, period: float, variance: EncoderMeasurement | None = None) -> None:
        super().__init__(period)
        self.variance = variance if variance is not None else EncoderMeasurement()

    def ground_truth(self, state: DifferentialDrive) -> EncoderMeasurement:
        return EncoderMeasurement(
            state.state.v,
            state.state.w,
        )
    
    def noisify(self, measurement: EncoderMeasurement) -> EncoderMeasurement:
        return EncoderMeasurement(
            gauss(measurement.v, sqrt(self.variance.v)),
            gauss(measurement.w, sqrt(self.variance.w))
        )


@dataclass
class PingerState:
    robot: Robot
    landmarks: Sequence[Vector]

@dataclass
class PingerMeasurement:
    pings: Sequence[Vector | None]

class Pinger(Sensor[PingerState, PingerMeasurement]):
    def __init__(self, range: float, period: float, variance: tuple[float, float] = (0.0, 0.0)) -> None:
        super().__init__(period)
        self.range = range
        self.variance = variance

    def ground_truth(self, state: PingerState) -> PingerMeasurement:
        pings: list[Vector] = []

        for landmark in state.landmarks:

            difference: Vector = landmark - state.robot.state.pos
            r = difference.r
            theta = difference.theta - state.robot.state.theta

            pings.append(Vector.polar(r, theta))

        return PingerMeasurement(pings)
    
    def noisify(self, measurement: PingerMeasurement) -> PingerMeasurement:
        noisy_pings = [
            Vector.polar(
                gauss(ping.r, sqrt(self.variance[0])),
                gauss(ping.theta, sqrt(self.variance[1]))
            ) if ping is not None else None for ping in measurement.pings
        ]

        ranged_pings = [
            ping if ping is not None and ping.r <= self.range 
            else None 
            for ping in noisy_pings
        ]

        return PingerMeasurement(ranged_pings)