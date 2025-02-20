import numpy as np
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter

# Initialiser le filtre de Kalman pour le suivi 2D (x, y, vx, vy)
kf = KalmanFilter(dim_x=4, dim_z=4)  # 4 états (x, y, vx, vy), 4 mesures (x, y, vx, vy)
dt = 1.0 # Intervalle de temps entre les mesures (ex: 1 seconde)
# 1. Matrice de transition (F) : Modèle de mouvement constant (x = x + v * dt)
kf.F = np.array([[1, 0, dt, 0],  # x = x + vx * dt
                 [0, 1, 0, dt],  # y = y + vy * dt
                 [0, 0, 1, 0],   # vx = vx
                 [0, 0, 0, 1]])  # vy = vy
# Matrice d'observation (H) : On fusionne deux capteurs (GPS + IMU)
kf.H = np.array([[1, 0, 0, 0],   # GPS -> x
                 [0, 1, 0, 0],   # GPS -> y
                 [0, 0, 1, 0],   # IMU -> vx
                 [0, 0, 0, 1]])  # IMU -> vy
kf.Q = np.eye(4) * 0.01 # 3. Bruit de processus (Q) : Incertitude sur le modèle (accélération non prise en compte)
kf.R = np.diag([5, 5, 1, 1])  # GPS (position incertaine) et IMU (vitesse plus précise) # Bruit de mesure (R) : Confiance dans chaque capteur
kf.x = np.array([[0], [0], [1], [1]])# 5. État initial (x) : On commence au point (0, 0) avec une vitesse de (1, 1)
kf.P = np.eye(4) * 1000 # 6. Covariance initiale (P) : Incertitude initiale élevée

# Simulation : Générer des mesures bruitées
n_steps = 50
true_positions = [(i, i + 2) for i in range(n_steps)]  # Mouvement réel (x, y)
true_velocities = [(1, 1) for _ in range(n_steps)]     # Vitesse réelle (vx, vy)

gps_measurements = [(x + np.random.randn() * 2, y + np.random.randn() * 2) for x, y in true_positions] # GPS : Mesure de la position avec du bruit élevé
imu_measurements = [(vx + np.random.randn() * 0.5, vy + np.random.randn() * 0.5) for vx, vy in true_velocities] # IMU : Mesure de la vitesse avec du bruit faible

# Appliquer le filtre de Kalman
estimations = []

for i in range(n_steps):
    kf.predict() # Prédiction
    # Fusion : Combiner les mesures (GPS + IMU)
    z = np.array([[gps_measurements[i][0]],   # x mesuré par le GPS
                  [gps_measurements[i][1]],   # y mesuré par le GPS
                  [imu_measurements[i][0]],   # vx mesuré par l'IMU
                  [imu_measurements[i][1]]])  # vy mesuré par l'IMU
    # Mise à jour avec les mesures
    kf.update(z)

    # Sauvegarder l'estimation
    estimations.append((kf.x[0, 0], kf.x[1, 0]))

true_x, true_y = zip(*true_positions)
gps_x, gps_y = zip(*gps_measurements)
est_x, est_y = zip(*estimations)

plt.figure(figsize=(10, 8))
plt.plot(true_x, true_y, 'g-', label="Position réelle")
plt.scatter(gps_x, gps_y, c='r', label="GPS (bruité)")
plt.plot(est_x, est_y, 'b-', label="Estimation Kalman (fusion)")
plt.xlabel("Position X")
plt.ylabel("Position Y")
plt.title("Fusion de capteurs (GPS + IMU) avec Kalman")
plt.legend()
plt.grid()
plt.show()
