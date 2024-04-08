import numpy as np
from scipy.optimize import linear_sum_assignment

def hungarian_algorithm(cost_matrix):
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    return row_ind, col_ind

# Example usage
formation = {
            "origin":[[0,0,-2],[0,3,-2],[0,6,-2],[-3,0,-2],[-3,3,-2],[-3,6,-2],[-6,0,-2],[-6,3,-2],[-6,6,-2]],
            "diamond":[[0,0,0],[-3,-3,0],[-3,3,0],[-6,-6,0],[-6,6,0],[-9,-9,0],[-9,-3,0],[-9,3,0],[-9,9,0]]
        }

origin_formation = formation["origin"]
target_formation = formation["diamond"]
origin_formation = np.array(origin_formation)
origin_formation = (origin_formation - origin_formation[0, :])[1:]
target_formation = np.array(target_formation)
target_formation = (target_formation - target_formation[0, :])[1:]

cost_matrix = np.linalg.norm(origin_formation[:, None] - target_formation, axis=2)
# cost_matrix = np.array([[5, 1, 1], [1, 0, 1 ], [1, 2, 0]])
row_ind, col_ind = hungarian_algorithm(cost_matrix)
print(row_ind, col_ind)
print(cost_matrix[row_ind, col_ind].sum())
# 按照col_ind的顺序排列
new_formation = np.array(target_formation)[col_ind]
print(new_formation)

col_ind = col_ind + 1
print(col_ind)