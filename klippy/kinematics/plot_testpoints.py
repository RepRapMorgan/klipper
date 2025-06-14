import numpy as np
import matplotlib.pyplot as plt

# Parameters for the bounding box and circle
square_width = 380
square_height = 380
circle_radius = 335

# Number of test points
num_points = 100

# Generate random points within the bounding box
np.random.seed(42)  # For reproducibility
X = np.random.uniform(-square_width/2, square_width/2, num_points)
Y = np.random.uniform(-square_height/2, square_height/2, num_points)

# Filter points to retain only those within the circle
filtered_points = [(x, y) for x, y in zip(X, Y) if x**2 + y**2 <= circle_radius**2]

# Convert list to array for easy plotting
filtered_points = np.array(filtered_points)

# Print the test points
print("Filtered Test Points (x, y):")
print(filtered_points)

# Visualize the points
plt.figure(figsize=(8, 8))
plt.plot(filtered_points[:, 0], filtered_points[:, 1], 'ro', label='Test Points')

# Plot the square boundary
square = plt.Rectangle((-square_width/2, -square_height/2), square_width, square_height, fill=None, edgecolor='blue', linestyle='--')
plt.gca().add_patch(square)

# Plot the circle boundary
circle = plt.Circle((0, 0), circle_radius, fill=None, edgecolor='green', linestyle='--')
plt.gca().add_patch(circle)

plt.xlabel('X (mm)')
plt.ylabel('Y (mm)')
plt.title('SCARA Robot Test Points within Morgan Bed Shape')
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.show()
