import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define grid
x = np.arange(0, 50, 1)
y = np.arange(0, 50, 1)
X, Y = np.meshgrid(x, y)

# Define obstacle parameters
obstacle = (25, 25)  # Center of the obstacle
r_obstacle = 5  # Obstacle radius

# Create smoother potential field for the obstacle only
Z_obstacle = np.zeros_like(X, dtype=float)

for i in range(len(x)):
    for j in range(len(y)):
        d_obstacle = np.sqrt((obstacle[0] - X[i, j]) ** 2 + (obstacle[1] - Y[i, j]) ** 2)

        # Create a stronger repulsion within the obstacle radius
        if d_obstacle <= r_obstacle:
            Z_obstacle[i, j] = -200  # Maximum repulsion inside obstacle
        else:
            # Smooth decay outside obstacle boundary using radius as scaling factor
            Z_obstacle[i, j] = -200 * r_obstacle / (d_obstacle ** 2)

# Create 3D plot with only the obstacle
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Limit z-axis range to reduce calculation time
ax.set_zlim(-200, 0)

surf = ax.plot_surface(X, Y, Z_obstacle, cmap='inferno', edgecolor='k', alpha=0.85)
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Potential Field")
ax.set_title("3D Obstacle Potential Field (r = {})".format(r_obstacle))

# Add a color bar to show the potential values
fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5)

# Show plot
plt.show()