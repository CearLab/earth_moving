import matplotlib.pyplot as plt
from shapely.geometry import Point, MultiPoint
from shapely.geometry.polygon import Polygon
import numpy as np

# Define a set of points
points = [Point(1, 1), Point(1.5, 5), Point(2, 0), Point(0, 2)]

# Create a MultiPoint object
multi_point = MultiPoint(points)

# Calculate the convex hull
convex_hull = multi_point.convex_hull

# Define a point
point = Point(3, 3)

# Calculate the distance from the convex hull to the point
distance = convex_hull.distance(point)

# Find the nearest point on the convex hull to the external point
nearest_point = convex_hull.exterior.interpolate(convex_hull.exterior.project(point))

print(f"The convex hull: {convex_hull}")
print(f"The point: {point}")
print(f"Distance from convex hull to point: {distance}")
print(f"Nearest point on convex hull: {nearest_point}")

# Extract x and y coordinates of the convex hull
hull_x, hull_y = convex_hull.exterior.xy

# Extract x and y coordinates of the points
points_x = [p.x for p in points]
points_y = [p.y for p in points]

# Plot the points
plt.scatter(points_x, points_y, color='blue', label='Points')

# Plot the convex hull
plt.plot(hull_x, hull_y, color='green', label='Convex Hull')

# Plot the point
plt.scatter([point.x], [point.y], color='red', label='External Point')

# Plot the nearest point on the convex hull
plt.scatter([nearest_point.x], [nearest_point.y], color='orange', label='Nearest Point on Hull')

# Draw a line from the point to the nearest point on the convex hull
plt.plot([point.x, nearest_point.x], [point.y, nearest_point.y], 'k--', label='Distance')

# Display the distance text
plt.text((point.x + nearest_point.x) / 2, (point.y + nearest_point.y) / 2, f'{distance:.2f}', fontsize=12, color='black')

# Set plot title and legend
plt.title('Convex Hull and Distance to External Point')
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')

# Show plot
plt.show()
