from abc import ABC, abstractmethod
from dataclasses import dataclass
from math import cos, sin, sqrt
from random import gauss
from typing import Generic, TypeVar

from utils import Pose, Vector

State = TypeVar("State", bound=Pose) 
Control = TypeVar("Control") 

class Robot(ABC, Generic[State, Control]):
    """
    An abstract class for robots
    """

    def __init__(self, state: State) -> None:
        """
        Create the robot
        """
        super().__init__()
        self.state = state

    @classmethod
    @abstractmethod
    def kinematics(cls, start: State, control: Control, dt: float) -> State:
        """
        Compute the new state from an old state and a control command over a timestep.
        This function is not noisy, it is pure kinematics

        Params:
            start: The initial state
            control: The command to follow
            dt: the length of the timestep
        
        Returns:
            the new state 
        """

    @abstractmethod
    def noisify(self, control: Control) -> Control:
        """
        Add noise to the control command

        Params:
            control: The command to add noise to
        
        Returns:
            the control command with noise
        """

    def step(self, control: Control, dt: float) -> State:
        """
        Calculate a new, noisy state for a given control command

        Params:
            control: The command to follow
            dt: the length of the timestep
        
        Returns:
            the new state 
        """
        return self.kinematics(self.state, self.noisify(control), dt)

@dataclass
class DifferentialDriveState(Pose):
    v: float = 0.0
    w: float = 0.0

@dataclass
class DifferentialDriveControl():
    v: float = 0.0
    w: float = 0.0

class DifferentialDrive(Robot[DifferentialDriveState, DifferentialDriveControl]):
    def __init__(self, 
        state: DifferentialDriveState | None = None, 
        variance: DifferentialDriveControl | None = None,
    ) -> None:
        super().__init__(state if state is not None else DifferentialDriveState())
        self.variance = variance if variance is not None else DifferentialDriveControl()
    
    @classmethod
    def kinematics(cls, start: DifferentialDriveState, control: DifferentialDriveControl, dt: float) -> DifferentialDriveState:
        delta: DifferentialDriveState

        if control.w == 0.0: 
            delta = DifferentialDriveState(
                Vector(
                    control.v * cos(start.theta) * dt,
                    control.v * sin(start.theta) * dt,
                ),
            )
        elif control.v == 0.0: # Not strictly necessary, but should improve performance slightly
            delta = DifferentialDriveState(
                Vector(), # No movement
                control.w * dt,
            )
        else:
            delta = DifferentialDriveState(
                Vector(
                    (control.v / control.w) * (sin(start.theta + control.w * dt) - sin(start.theta)),
                    -(control.v / control.w) * (cos(start.theta + control.w * dt) - cos(start.theta))
                ),
                control.w * dt,
            )

        return DifferentialDriveState(
            start.pos + delta.pos,
            start.theta + delta.theta,
            control.v,
            control.w
        )

    def noisify(self, control: DifferentialDriveControl) -> DifferentialDriveControl:
        return DifferentialDriveControl(
            gauss(control.v, sqrt(self.variance.v)),
            gauss(control.w, sqrt(self.variance.w))
        ) 

@dataclass
class HolonomicDriveState(Pose):
    xv: float = 0.0
    yv: float = 0.0
    w: float = 0.0

@dataclass
class HolonomicDriveControl():
    xv: float = 0.0
    yv: float = 0.0
    w: float = 0.0

class HolonomicDrive(Robot[HolonomicDriveState, HolonomicDriveControl]):
    def __init__(self, state: HolonomicDriveState | None = None, variance: HolonomicDriveControl | None = None) -> None:
        super().__init__(state if state is not None else HolonomicDriveState())
        self.variance = variance if variance is not None else HolonomicDriveControl()
    
    @classmethod
    def kinematics(cls, start: HolonomicDriveState, control: HolonomicDriveControl, dt: float) -> HolonomicDriveState:
        return HolonomicDriveState(
            start.pos + Vector(control.xv * dt, control.yv * dt),
            start.theta + control.w * dt,
            control.xv,
            control.yv,
            control.w
        )

    def noisify(self, control: HolonomicDriveControl) -> HolonomicDriveControl:
        return HolonomicDriveControl(
            gauss(control.xv, sqrt(self.variance.xv)),
            gauss(control.yv, sqrt(self.variance.yv)),
            gauss(control.w, sqrt(self.variance.w))
        ) 
