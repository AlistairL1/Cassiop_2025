import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define grid
x = np.arange(0, 50, 1)
y = np.arange(0, 50, 1)
X, Y = np.meshgrid(x, y)

# Define obstacle parameters
obstacle = (25, 25)  # Center of the obstacle
r_obstacle = 2  # Obstacle radius

# Variance parameter (σ²) - controls the spread of the gaussian
variance = 40  # Try adjusting this value (smaller=steeper, larger=wider spread)

# Create potential field using multivariate gaussian
Z_obstacle = np.zeros_like(X, dtype=float)

# Amplitude parameter for the potential field
A = -50  # Negative for repulsive field

# Create gaussian potential field
for i in range(len(x)):
    for j in range(len(y)):
        # Distance from point to obstacle center
        dx = X[i, j] - obstacle[0]
        dy = Y[i, j] - obstacle[1]

        # Inside the obstacle: constant maximum repulsion
        d_obstacle = np.sqrt(dx ** 2 + dy ** 2)
        if d_obstacle <= r_obstacle:
            Z_obstacle[i, j] = A
        else:
            # Gaussian function: A * exp(-((x-μ)²/(2σ²)))
            Z_obstacle[i, j] = A * np.exp(-(dx ** 2 + dy ** 2) / (2 * variance)) * (r_obstacle / d_obstacle)

# Create 3D plot with only the obstacle
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Limit z-axis range to reduce calculation time
ax.set_zlim(A, 0)  # From A to 0

surf = ax.plot_surface(X, Y, Z_obstacle, cmap='inferno', edgecolor='none', alpha=0.85)
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Potential Field")
ax.set_title("Gaussian Obstacle Potential Field (r = {}, σ² = {})".format(r_obstacle, variance))

# Add a color bar to show the potential values
fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5)

# Add a 2D contour plot at the bottom to better visualize the gaussian shape
cset = ax.contour(X, Y, Z_obstacle, zdir='z', offset=A, cmap='inferno')

# Show plot
plt.show()