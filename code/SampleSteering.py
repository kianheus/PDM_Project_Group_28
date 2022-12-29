# This code shows how to use the Dubins Path generator in Steering.py

import Steering as steer
import numpy as np
import time

n = 10000
t_s = time.time()

# for i in range(n):
# define a starting point
point_start = np.array([0.0, 3.0, np.deg2rad(20)])

# define a ending point
point_end = np.array([1.0, 0.8, np.deg2rad(-70)])

# define a minimum turning radius (can also be a list of multiple angles)
radius_min = 0.5

# calculate the shortest points
shortest_path = steer.optimal_path(point_start, point_end, radius_min)

t_e = time.time()
print(f"time = {(t_e-t_s)}")

# extract a list of points (n evenly spaced points)
points_n = shortest_path.interpolate(n=10)

# extract a list of points (spaced a distance of roughly d units appart)
points_d = shortest_path.interpolate(d=0.33)

# get the length of the path
length = shortest_path.length

print(f"{points_n=}")
print(f"{points_d=}")
print(f"{length=}")


# The first point in the path list is equal to the starting point (within floating point error)
# This is also guaranteed when using distance spacing
print(f"{points_n[0,:]} == {point_start}")

# The last point in the path list is equal to the ending point (within floating point error)
print(f"{points_n[-1,:]} == {point_end}")
