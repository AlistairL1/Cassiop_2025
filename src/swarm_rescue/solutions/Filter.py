import numpy as np
from filterpy.kalman import KalmanFilter


class Filter:
    def __init__(self):
        self.previous = None
        self.window_size = 0
        self.window = []


    def low_pass(self, current_value, alpha):
        self.previous = None

        if self.previous is None:
            # Initialize the filter with the first value
            self.previous = current_value
            return current_value

        filtered_value = alpha * current_value + (1 - alpha) * self.previous
        self.previous = filtered_value
        return filtered_value

    def moving_mean(self, value):
        self.window_size = 10
        self.window.append(value)
        # If the window exceeds the allowed size, remove the oldest value
        if len(self.window) > self.window_size:
            self.window.pop(0)
        return self.window

    def error(self, true, noise):
        return np.linalg.norm(true - noise)

    def Kalman(self, gps_value, imu_value):
        # Initialiser le filtre de Kalman pour le suivi 2D (x, y, vx, vy)
        kf = KalmanFilter(dim_x=4, dim_z=4)  # 4 états (x, y, vx, vy), 4 mesures (x, y, vx, vy)
        dt = 1.0  # Intervalle de temps entre les mesures
        # Matrice de transition (F) : Modèle de mouvement constant (x = x + v * dt)
        kf.F = np.array([[1, 0, dt, 0],  # x = x + vx * dt
                         [0, 1, 0, dt],  # y = y + vy * dt
                         [0, 0, 1, 0],  # vx = vx
                         [0, 0, 0, 1]])  # vy = vy
        # Matrice d'observation (H) : On fusionne deux capteurs (GPS + IMU)
        kf.H = np.array([[1, 0, 0, 0],  # GPS -> x
                         [0, 1, 0, 0],  # GPS -> y
                         [0, 0, 1, 0],  # IMU -> vx
                         [0, 0, 0, 1]])  # IMU -> vy
        kf.Q = np.eye(4) * 0.01  # 3. Bruit de processus (Q) : Incertitude sur le modèle (accélération non prise en compte)
        kf.R = np.diag([5, 5, 1, 1])  # GPS (position incertaine) et IMU (vitesse plus précise) # Bruit de mesure (R) : Confiance dans chaque capteur
        kf.x = np.array([[0], [0], [0], [0]])  # 5. État initial (x) : On commence au point (0, 0) avec une vitesse de (0, 0)
        kf.P = np.eye(4) * 1000  # 6. Covariance initiale (P) : Incertitude initiale élevée

        gps_measurements = gps_value  # GPS : Mesure de la position avec du bruit
        imu_measurements = imu_value  # IMU : Mesure de la vitesse avec du bruit

        # Appliquer le filtre de Kalman
        estimations = []
        #n_steps = 50
        #for i in range(n_steps):
        kf.predict()  # Prédiction
            # Fusion : Combiner les mesures (GPS + IMU)
        z = np.array([[gps_measurements[0]],  # x mesuré par le GPS
                        [gps_measurements[1]],  # y mesuré par le GPS
                        [imu_measurements[0]],  # vx mesuré par l'IMU
                        [imu_measurements[1]]])  # vy mesuré par l'IMU
            # Mise à jour avec les mesures
        kf.update(z)

            # Sauvegarder l'estimation
        estimations.append((kf.x[0, 0], kf.x[1, 0]))

        #gps_x, gps_y = zip(*gps_measurements)
        #est_x, est_y = zip(*estimations)
        return estimations