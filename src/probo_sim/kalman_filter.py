from dataclasses import dataclass
import numpy as np
from typing import Sequence

@dataclass(kw_only=True)
class KalmanFilter:
    """
    A class for performing Kalman filtering.
    """
    transition_matrix: np.ndarray
    control_matrix: np.ndarray
    measurement_matrix: np.ndarray
    process_noise: np.ndarray
    measurement_noise: np.ndarray
    
    def predict(self, x: np.ndarray, P: np.ndarray, control: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        x_hat = self.transition_matrix @ x + self.control_matrix @ control
        P_hat = self.transition_matrix @ P @ self.transition_matrix.T + self.process_noise
        return x_hat, P_hat
    
    def update(self, x_hat: np.ndarray, P_hat: np.ndarray, measurement: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        y = measurement - self.measurement_matrix @ x_hat
        S = self.measurement_matrix @ P_hat @ self.measurement_matrix.T + self.measurement_noise
        K = P_hat @ self.measurement_matrix.T @ np.linalg.inv(S)
        x = x_hat + K @ y
        P = P_hat - K @ self.measurement_matrix @ P_hat

        return x, P
    
    def step(self, x: np.ndarray, P: np.ndarray, control: np.ndarray, measurement: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        x_hat, P_hat = self.predict(x, P, control)
        x, P = self.update(x_hat, P_hat, measurement)
        return x, P

    def run(self, prior: np.ndarray, initial_P: np.ndarray, controls: Sequence[np.ndarray], measurements: Sequence[np.ndarray]) -> tuple[list[np.ndarray], list[np.ndarray]]:
        x: list[np.ndarray] = []
        P: list[np.ndarray] = []

        steps = min(len(controls), len(measurements))

        for step in range(steps):
            previous_x = prior if step == 0 else x[step-1]
            previous_P = initial_P if step == 0 else P[step-1]

            next_x, next_P = self.step(previous_x, previous_P, controls[step], measurements[step])
            x.append(next_x)
            P.append(next_P)

        return x, P