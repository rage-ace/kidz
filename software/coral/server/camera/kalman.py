import time
import numpy as np


class KalmanFilter:
    def __init__(self, F=None, B=None, H=None, Q=None, R=None, P=None, x0=None) -> None:
        if F is None or H is None:
            raise ValueError("Set proper system dynamics.")

        # Initialise parameters
        self.n = F.shape[1]
        self.m = H.shape[1]
        self.F = F
        self.H = H
        # self.B = np.zeros(1, dtype=np.float) if B is None else B
        self.Q = np.eye(self.n, dtype=np.float) if Q is None else Q
        self.R = np.eye(self.n, dtype=np.float) if R is None else R
        self.P = np.zeros((self.n, self.n)) if P is None else P
        self.x = None

        self._last_time = None

    def predict(self) -> np.ndarray:
        # Compute dt at each prediction step
        if self._last_time is None:
            self._last_time = time.time()
            return None
        current_time = time.time()
        dt = current_time - self._last_time
        self._last_time = current_time
        self.F[0][2] = dt
        self.F[1][3] = dt

        # Predict
        self.x = np.dot(self.F, self.x)  # + np_dot( self.B, u )
        self.P = np.dot(np.dot(self.F, self.P), self.F.transpose().copy()) + self.Q
        return self.x

    def update(self, z) -> None:
        # Initialise parameters if it's the first update
        if self.P is None:
            self.P = np.eye(self.F.shape[1])
        if self.x is None:
            self.x = np.array([z[0], z[1], [0], [0], z[2], z[3]], dtype=np.float)

        # Update state
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.transpose().copy()))
        K = np.dot(np.dot(self.P, self.H.transpose().copy()), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(
            np.dot(I - np.dot(K, self.H), self.P),
            (I - np.dot(K, self.H)).transpose().copy(),
        ) + np.dot(np.dot(K, self.R), K.transpose().copy())
