from utils import Bounds

class Environment:
    def __init__(self,
        bounds: Bounds = Bounds(),
        obstacles: list[Bounds] = [],
    ) -> None:
        self.bounds = bounds
        self.obstacles = obstacles
