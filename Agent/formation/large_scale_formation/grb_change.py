import gurobipy as gp
from gurobipy import GRB
import numpy as np
import sys
sys.path.insert(1,".")
from Agent.formation.formation_dict import formation_dict_9, formation_dict_8, formation_dict_6
import random
size = 100
random.seed(42)

# formation = formation_dict_9
# origin_formation = formation["origin"]
# target_formation = formation["triangle"]

origin_formation = []
target_formation = []

for i in range(size):
    loc1 = [random.randint(0,100),random.randint(0,100),random.randint(0,100)]
    loc2 = [random.randint(0,100),random.randint(0,100),random.randint(0,100)]
    origin_formation.append(loc1)
    target_formation.append(loc2)

adj_matrix = np.linalg.norm(np.array(origin_formation)[:, None] - np.array(target_formation), axis=2)
size = adj_matrix.shape[0]
adj_matrix_dict = {(i, j): adj_matrix[i][j] for i in range(size) for j in range(size)}
# Establish the model
m = gp.Model('Assignment_problem')

# Add decision variables
x = m.addVars(adj_matrix_dict.keys(), vtype=GRB.BINARY, name='x')
t = m.addVar(lb = 0, vtype=GRB.CONTINUOUS, name = "t")
# Add constraints
m.addConstrs((x.sum(i, '*') == 1 for i in range(size)), 'Constraint_of_worker')
m.addConstrs((x.sum('*', j) == 1 for j in range(size)), 'Constraint_of_job')
m.addConstrs((x[i,j]*adj_matrix_dict[i,j] <= t for i in range(size) for j in range(size)), 'Constraint_of_Bound')

k = 2*size
m.setObjective(x.prod(adj_matrix_dict)+k*t,GRB.MINIMIZE)

# m.setObjective(x.prod(adj_matrix_dict)+t,GRB.MINIMIZE)
# m.setObjectiveN(x.prod(T), index=0, priority=1, abstol=0, reltol=0, name='minimize_time')
# m.setObjectiveN(-x.prod(C), index=1, priority=2, abstol=100, reltol=0, name='maximize_profit')

# Solve the model
m.optimize()

origin_ids = [i for i in range(len(origin_formation))]
change_ids = {}
costsum = 0
for i in range(size):
    for j in range(size):
        if x[i,j].x == 1:
            change_ids[origin_ids[i]] = j
            costsum += adj_matrix_dict[i,j]
print(f"\nruntime {m.Runtime}, milp costsum {costsum}, costmax {t.x}")
# print(sum(adj_matrix))
# print("\nThe objective function values are:")
# print('Obj1: the total time is: {}'.format(m.ObjVal))

