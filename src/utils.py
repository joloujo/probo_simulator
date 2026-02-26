from dataclasses import dataclass, field
from math import atan2, cos, sin, hypot

@dataclass
class Vector:
    """
    A two dimensional vector
    """

    x: float = 0.0
    y: float = 0.0

    @classmethod
    def polar(cls, r: float, theta: float) -> 'Vector':
        """
        Create a vector from it's polar coordinates
        """
        return Vector(r * cos(theta), r * sin(theta))

    def __str__(self) -> str:
        """
        Convert the vector to a string
        """
        return f'({self.x}, {self.y})'
    
    def __add__(self, other: 'Vector') -> 'Vector':
        """
        Add two vectors
        """
        return Vector(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other: 'Vector') -> 'Vector':
        """
        Subtract two vectors
        """
        return Vector(self.x - other.x, self.y - other.y)
    
    def __mul__(self, other: int | float) -> 'Vector':
        """
        Scale a vector
        """
        return Vector(self.x * other, self.y * other)

    @property
    def r(self) -> float:
        """
        The magnitude of the vector
        """
        return hypot(self.x, self.y)
    
    @property
    def theta(self) -> float:
        """
        The angle of the vector. Returns 0.0 if undefined
        """
        if self.x == 0.0 and self.y == 0.0:
            return 0.0
        return atan2(self.y, self.x)
    
    def copy(self) -> 'Vector':
        """
        Copy the vector
        """
        return Vector(self.x, self.y)

def new_vector() -> Vector:
    return Vector(0.0, 0.0)

@dataclass
class Pose:
    """
    A position and orientation
    """
    pos: Vector = field(default_factory=new_vector)
    theta: float = 0.0

    def __str__(self) -> str:
        """
        Convert the pose to a string
        """
        return f'({self.pos.x}, {self.pos.y}, {self.theta})'
    
    def copy(self) -> 'Pose':
        """
        Copy the pose
        """
        return Pose(self.pos.copy(), self.theta)

@dataclass
class Bounds:
    """
    A rectangular region
    """
    min: Vector = field(default_factory=new_vector)
    max: Vector = field(default_factory=new_vector)

    # TODO: Add protection for mixing up min and max values

    def contains(self, vec: Vector) -> bool:
        """
        Return if a location is inside the bounds
        """
        return self.min.x <= vec.x <= self.max.x and self.min.y <= vec.y <= self.max.y
