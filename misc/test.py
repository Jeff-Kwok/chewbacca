import numpy as np
avoid_distances = {
    "close":2.0,
    "middle":4.0,
    "far":6.0
}
distance = 5.6
MAX_RANGE_METERS = 12.0
for val,param in enumerate(avoid_distances):
    if not round(distance,2) >= avoid_distances[param]:
        # For example if we detect an object within the middle range we divide it by our max detect distance to give 
        # us a ratio of our maximum allowable speed

        speed = np.interp(distance,[0.0,MAX_RANGE_METERS],[0.0,avoid_distances["far"]])
        print(speed)
        # send payload to slow vehicle down