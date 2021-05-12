import numpy as np

def vectorize_knee_orientation(knee_orientation):
    def get_knee_direction(direction):
        if direction == '>':
            return -1
        elif direction == '<':
            return 1
        else:
            return -1

    knee_directions = np.zeros((4,1))

    for i in range(4):
        if i < 2:
            direction = get_knee_direction(knee_orientation[0])
        else:
            direction = get_knee_direction(knee_orientation[1])

        knee_directions[i, 0]  = direction

    return knee_directions

def clip(val, min_val, max_val):
    if val < 0 and val < min_val:
        return min_val
    elif val > 0 and val > max_val:
        return max_val
    else:
        return val