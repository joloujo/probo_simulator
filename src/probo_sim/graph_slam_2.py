from abc import ABC
import numpy as np
import sympy
from typing import Any

from probo_sim.utils import Pose, Vector

class Factor(ABC):
    def __init__(self, errors: list[Any], variables) -> None:
        residuals = sympy.Matrix(errors)
        self._residuals = sympy.lambdify(variables, residuals)

        cost = 0.5 * sum([error ** 2 for error in errors]) # type: ignore
        self._cost = sympy.lambdify(variables, cost, "numpy")

        jacobian = residuals.jacobian(variables)
        self._jacobian = sympy.lambdify(variables, jacobian, "numpy")

        hessian = sympy.hessian(cost, variables) 
        self._hessian = sympy.lambdify(variables, hessian, "numpy")

        approximate_hessian = jacobian.T @ jacobian 
        self._approximate_hessian = sympy.lambdify(variables, approximate_hessian, "numpy")

    def residuals(self, state: np.ndarray) -> np.ndarray:
        return self._residuals(*state)[:, 0]

    def cost(self, state: np.ndarray) -> float:
        return self._cost(*state)
    
    def gradient(self, state: np.ndarray) -> np.ndarray:
        return (self.jacobian(state).T @ self.residuals(state)).T
    
    def jacobian(self, state: np.ndarray) -> np.ndarray:
        return self._jacobian(*state)
    
    def hessian(self, state: np.ndarray) -> np.ndarray:
        return self._hessian(*state)
    
    def approximate_hessian(self, state: np.ndarray) -> np.ndarray:
        return self._approximate_hessian(*state)

class OdomFactor(Factor):
    def __init__(self, measurement: np.ndarray) -> None:
        x_a, y_a, t_a, x_b, y_b, t_b = sympy.symbols('x_a, y_a, t_a, x_b, y_b, t_b')
        variables = sympy.Matrix([x_a, y_a, t_a, x_b, y_b, t_b])

        error_x = sympy.cos(t_a) * (x_b - x_a) + sympy.sin(t_a) * (y_b - y_a) - measurement[0]
        error_y = -1 * sympy.sin(t_a) * (x_b - x_a) + sympy.cos(t_a) * (y_b - y_a) - measurement[1] # type: ignore
        error_theta = sympy.atan2(sympy.sin(t_b - t_a - - measurement[2]), sympy.cos(t_b - t_a - - measurement[2]))

        super().__init__([error_x, error_y, error_theta], variables)
        
class PingFactor(Factor):
    def __init__(self, measurement: np.ndarray) -> None:
        x_a, y_a, t_a, x_b, y_b = sympy.symbols('x_a, y_a, t_a, x_b, y_b')
        variables = sympy.Matrix([x_a, y_a, t_a, x_b, y_b])

        error_x = sympy.cos(t_a) * (x_b - x_a) + sympy.sin(t_a) * (y_b - y_a) - measurement[0]
        error_y = -1 * sympy.sin(t_a) * (x_b - x_a) + sympy.cos(t_a) * (y_b - y_a) - measurement[1] # type: ignore

        super().__init__([error_x, error_y], variables)


class GraphSLAM():
    def __init__(self, n: int) -> None:
        self.n = n
        self.state = np.zeros(n)
        self.factors: list[tuple[list[int], Factor]] = []

    def add_factor(self, indexes: list[int], factor: Factor):
        self.factors.append((indexes, factor))

    def gradient_descent_step(self, alpha: float) -> float:
        gradient = np.zeros(self.n)

        for indexes, factor in self.factors:
            gradient[indexes] += factor.gradient(self.state[indexes])

        self.state -= gradient * alpha

        return sum([
            factor.cost(self.state[indexes])
            for indexes, factor in self.factors
        ])


# gs = GraphSLAM(8)

# gs.add_factor([0, 1, 2, 3, 4, 5], OdomFactor(np.array([1, 0, 0])))
# gs.add_factor([0, 1, 2, 6, 7], PingFactor(np.array([0, 1])))
# gs.add_factor([3, 4, 5, 6, 7], PingFactor(np.array([-1, 1])))

# for i in range(10):
#     gs.gradient_descent_step(0.5)

# print(gs.state)
