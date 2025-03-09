import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def calculate_gradient_field(Z, X, Y):
    """
    Calculate the gradient of the potential field and find minimum gradient point.

    Args:
        Z: The potential field values
        X, Y: The meshgrid coordinates

    Returns:
        grad_x, grad_y: The gradient components
        min_grad_point: Coordinates of minimum gradient magnitude point
        no_field: Boolean indicating if there's no meaningful gradient field
    """
    # Calculate gradient using numpy's gradient function
    grad_y, grad_x = np.gradient(Z)

    # Calculate gradient magnitude
    grad_magnitude = np.sqrt(grad_x ** 2 + grad_y ** 2)

    # Check if there's a meaningful field
    if np.any(grad_magnitude > 0.001):  # Small threshold to account for numerical precision
        # Find the minimum non-zero gradient
        # Create a masked array to ignore zero or near-zero gradients
        masked_magnitude = np.ma.masked_where(grad_magnitude < 0.001, grad_magnitude)

        # Find the minimum gradient
        min_idx = np.ma.argmin(masked_magnitude)
        min_i, min_j = np.unravel_index(min_idx, grad_magnitude.shape)
        min_grad_point = (X[min_i, min_j], Y[min_i, min_j])

        return grad_x, grad_y, min_grad_point, False
    else:
        # No meaningful field case
        return grad_x, grad_y, None, True


# Define grid
x = np.arange(0, 50, 1)
y = np.arange(0, 50, 1)
X, Y = np.meshgrid(x, y)

# Define obstacle parameters
obstacle = (25, 25)  # Center of the obstacle
r_obstacle = 3  # Obstacle radius - increased for better visualization

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

# Calculate gradient field
grad_x, grad_y, min_point, no_field = calculate_gradient_field(Z_obstacle, X, Y)

# Create figure with two subplots: 3D surface and 2D gradient field
fig = plt.figure(figsize=(16, 8))

# 3D surface plot
ax1 = fig.add_subplot(121, projection='3d')
ax1.set_zlim(-200, 0)
surf = ax1.plot_surface(X, Y, Z_obstacle, cmap='inferno', edgecolor='k', alpha=0.85)
ax1.set_xlabel("X-axis")
ax1.set_ylabel("Y-axis")
ax1.set_zlabel("Potential Field")
ax1.set_title("3D Obstacle Potential Field (r = {})".format(r_obstacle))
fig.colorbar(surf, ax=ax1, shrink=0.5, aspect=5)

# 2D gradient plot
ax2 = fig.add_subplot(122)
# Plot gradient field as quiver plot
step = 2  # Skip some points to make the quiver plot clearer
ax2.quiver(X[::step, ::step], Y[::step, ::step],
           grad_x[::step, ::step], grad_y[::step, ::step],
           color='blue', scale=50)

# Add obstacle visualization
circle = plt.Circle(obstacle, r_obstacle, color='red', alpha=0.7)
ax2.add_patch(circle)

# If there's a minimum gradient point, mark it
if not no_field and min_point:
    ax2.plot(min_point[0], min_point[1], 'go', markersize=10)
    ax2.annotate('Min gradient', xy=min_point, xytext=(min_point[0] + 3, min_point[1] + 3),
                 arrowprops=dict(facecolor='green', shrink=0.05))

ax2.set_xlabel("X-axis")
ax2.set_ylabel("Y-axis")
ax2.set_title("Gradient Field and Minimum Gradient Point")
ax2.set_xlim(0, 50)
ax2.set_ylim(0, 50)

plt.tight_layout()
plt.show()