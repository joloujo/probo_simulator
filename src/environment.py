from utils import Bounds

class Environment:
    """
    A class to represent the environment in a simulation
    """
    def __init__(self,
        bounds: Bounds = Bounds(),
        obstacles: list[Bounds] = [],
    ) -> None:
        """
        Create the environment

        Params:
            bounds: The limits of the environment
            obstacles: a list of obstacles in the environment
        """
        self.bounds = bounds
        self.obstacles = obstacles
