from abc import ABC
import numpy as np
import sympy

class Factor(ABC):

    _state_variables: sympy.Matrix
    _measurement_variables: sympy.Matrix
    _error_functions: sympy.Matrix

    measurement: np.ndarray
    fim: np.ndarray

    def __init__(self) -> None:
        residuals = sympy.Matrix(self._error_functions)
        self._residuals = sympy.lambdify((self._state_variables, self._measurement_variables), residuals)

        cost = residuals.T @ self.fim @ residuals
        self._cost = sympy.lambdify((self._state_variables, self._measurement_variables), cost, "numpy")

        jacobian = residuals.jacobian(self._state_variables)
        self._jacobian = sympy.lambdify((self._state_variables, self._measurement_variables), jacobian, "numpy")

        hessian = sympy.hessian(cost, self._state_variables) 
        self._hessian = sympy.lambdify((self._state_variables, self._measurement_variables), hessian, "numpy")

        approximate_hessian = jacobian.T @ self.fim @ jacobian 
        self._approximate_hessian = sympy.lambdify((self._state_variables, self._measurement_variables), approximate_hessian, "numpy")

    def residuals(self, state: np.ndarray) -> np.ndarray:
        return self._residuals(state, self.measurement)[:, 0]

    def cost(self, state: np.ndarray) -> float:
        return self._cost(state, self.measurement)[0, 0]
    
    def gradient(self, state: np.ndarray) -> np.ndarray:
        return (self.jacobian(state).T @ self.fim @ self.residuals(state)).T
    
    def jacobian(self, state: np.ndarray) -> np.ndarray:
        return self._jacobian(state, self.measurement)
    
    def hessian(self, state: np.ndarray) -> np.ndarray:
        return self._hessian(state, self.measurement)
    
    def approximate_hessian(self, state: np.ndarray) -> np.ndarray:
        return self._approximate_hessian(state, self.measurement)

class OdomFactor(Factor):

    x_a, y_a, t_a, x_b, y_b, t_b = sympy.symbols('x_a, y_a, t_a, x_b, y_b, t_b')
    _state_variables = sympy.Matrix([x_a, y_a, t_a, x_b, y_b, t_b])
    
    x_m, y_m, t_m = sympy.symbols('x_m, y_m, t_m')
    _measurement_variables = sympy.Matrix([x_m, y_m, t_m])

    error_x = sympy.cos(t_a) * (x_b - x_a) + sympy.sin(t_a) * (y_b - y_a) - x_m
    error_y = -1 * sympy.sin(t_a) * (x_b - x_a) + sympy.cos(t_a) * (y_b - y_a) - y_m # type: ignore
    error_theta = sympy.atan2(sympy.sin(t_b - t_a - t_m), sympy.cos(t_b - t_a - t_m))
    _error_functions = sympy.Matrix([error_x, error_y, error_theta])

    def __init__(self, measurement: np.ndarray, covariance: np.ndarray = np.identity(3)) -> None:
        self.measurement = measurement
        self.fim = np.linalg.inv(covariance)
        
        super().__init__()
        
class PingFactor(Factor):

    x_a, y_a, t_a, x_b, y_b = sympy.symbols('x_a, y_a, t_a, x_b, y_b')
    _state_variables = sympy.Matrix([x_a, y_a, t_a, x_b, y_b])

    x_m, y_m = sympy.symbols('x_m, y_m')
    _measurement_variables = sympy.Matrix([x_m, y_m])

    error_x = sympy.cos(t_a) * (x_b - x_a) + sympy.sin(t_a) * (y_b - y_a) - x_m
    error_y = -1 * sympy.sin(t_a) * (x_b - x_a) + sympy.cos(t_a) * (y_b - y_a) - y_m # type: ignore
    _error_functions = sympy.Matrix([error_x, error_y])

    def __init__(self, measurement: np.ndarray, covariance: np.ndarray = np.identity(2)) -> None:
        self.measurement = measurement
        self.fim = np.linalg.inv(covariance)

        super().__init__()

class PriorFactor(Factor):

    x, y, t = sympy.symbols('x, y, t')
    _state_variables = sympy.Matrix([x, y, t])

    x_m, y_m, t_m = sympy.symbols('x_m, y_m, t_m')
    _measurement_variables = sympy.Matrix([x_m, y_m, t_m])

    error_x = (x - x_m)
    error_y = (y - y_m)
    error_t = (t - t_m)
    _error_functions = sympy.Matrix([error_x, error_y, error_t])

    def __init__(self, measurement: np.ndarray, covariance: np.ndarray = np.identity(3)) -> None:
        self.measurement = measurement
        self.fim = np.linalg.inv(covariance)
    
        super().__init__()


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

    def gauss_newton_step(self) -> float:
        gradient = np.zeros(self.n)
        hessian = np.zeros((self.n, self.n))

        for indexes, factor in self.factors:
            gradient[indexes] += factor.gradient(self.state[indexes])
            hessian[np.ix_(indexes, indexes)] += factor.approximate_hessian(self.state[indexes])

        self.state += np.linalg.solve(hessian, -gradient)

        return sum([
            factor.cost(self.state[indexes])
            for indexes, factor in self.factors
        ])
