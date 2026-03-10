from abc import ABC
import numpy as np
import sympy
from typing import Any

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
        return self._residuals(*state)

    def cost(self, state: np.ndarray) -> float:
        return self._cost(*state)
    
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


# T = TypeVar("T", bound=tuple[Factor, ...])

# class GraphSLAM[T]():
#     def __init__(self, nodes: T) -> None:
#         self.nodes = nodes

#         self.state




o = OdomFactor(
    np.array([1, 0, 0])
)

o_state = np.array([0, 0, 0, 0.9, 0, 0])

print(o.cost(o_state))
print(o.jacobian(o_state))
print(o.jacobian(o_state) * o.residuals(o_state))
print(o.hessian(o_state))
print(o.hessian(o_state) - o.approximate_hessian(o_state))


p = PingFactor(
    np.array([1, 1])
)

p_state = np.array([0, 0, 0, 1.1, 0.9])

print(p.cost(p_state))
print(p.jacobian(p_state))
print(p.jacobian(p_state) * p.residuals(p_state))
print(p.hessian(p_state))
print(p.hessian(p_state) - p.approximate_hessian(p_state))





