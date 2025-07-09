from filterpy.kalman import KalmanFilter
import numpy as np

class ConeKF:
    def __init__(self, initial_pos, dt=0.1, process_var=1e-2, meas_var=1e-1):
        # 6D state: [x, y, z, vx, vy, vz]
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        # State transition: constant velocity
        self.kf.F = np.array([
            [1,0,0, dt,0,0],
            [0,1,0, 0,dt,0],
            [0,0,1, 0,0,dt],
            [0,0,0, 1,0,0],
            [0,0,0, 0,1,0],
            [0,0,0,  0,0,1],
        ])
        # Measurement function: we measure only position
        self.kf.H = np.array([
            [1,0,0, 0,0,0],
            [0,1,0, 0,0,0],
            [0,0,1, 0,0,0],
        ])
        # Covariances
        self.kf.P[:] = np.eye(6) * 1.0        # initial covariance
        self.kf.Q[:] = np.eye(6) * process_var # process noise
        self.kf.R[:] = np.eye(3) * meas_var    # measurement noise

        # initialize state vector
        x, y, z = initial_pos
        self.kf.x = np.array([x, y, z, 0, 0, 0], dtype=float)

    def predict(self, dt=None):
        if dt is not None:
            # update F if your loop time is variable
            self.kf.F[0,3] = self.kf.F[1,4] = self.kf.F[2,5] = dt
        self.kf.predict()
        return self.kf.x[:3].copy()

    def update(self, measurement):
        self.kf.update(measurement)
        return self.kf.x[:3].copy()
