import gurobipy as gp
from gurobipy import GRB
import numpy as np
import sys
sys.path.insert(1,".")
from Agent.formation.formation_dict import formation_dict_9, formation_dict_8, formation_dict_6
import random

from Agent.formation.large_scale_formation.generate_tri import generate_triangle_points
from Agent.formation.large_scale_formation.generate_des_tri import generate_dense_triangle_points
from Agent.formation.large_scale_formation.generate_circle import generate_circle_points
from Agent.formation.large_scale_formation.generate_des_rec import generate_dense_rectangle_points

# 导入队形
# formation = formation_dict_9
# origin_formation = formation["origin"]
# target_formation = formation["triangle"]

# 生成指定队形
# origin_formation = generate_circle_points(leader_point = [0,0,0], num_points = 11, direction= [1,0,0], spacing= 3)
# target_formation = generate_triangle_points(leader_point = [0,0,0], num_points = 100, direction= [1,0,0], spacing= 3)


# 不同队形切换
origin_formation = generate_dense_rectangle_points(leader_point = [0,0,0], num_points = 25, direction= [0,1,0], spacing= 3)
target_formation = generate_dense_triangle_points(leader_point = [0,0,0], num_points = 25, direction= [0,1,0], spacing= 3)


# 队形修复测试
# origin_formation = generate_dense_triangle_points(leader_point = [0,0,0], num_points = 35, direction= [1,0,0], spacing= 3)
# # target_formation = generate_circle_points(leader_point = [0,0,0], num_points = 20, direction= [1,0,0], spacing= 3)
# target_formation = generate_dense_triangle_points(leader_point = [0,0,0], num_points = 33, direction= [1,0,0], spacing= 3)
# origin_formation = np.delete(origin_formation,1,0)
# origin_formation = np.delete(origin_formation,15,0)


# 随机生成队形
# size = 10
# random.seed(42)
# for i in range(size):
#     loc1 = [random.randint(0,100),random.randint(0,100),random.randint(0,100)]
#     loc2 = [random.randint(0,100),random.randint(0,100),random.randint(150,200)]
#     origin_formation.append(loc1)
#     target_formation.append(loc2)

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
m.setObjective(x.prod(adj_matrix_dict),GRB.MINIMIZE)
# m.setObjective(x.prod(adj_matrix_dict),GRB.MINIMIZE)
# m.setObjective(t,GRB.MINIMIZE)
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


# 实际队形
real_target_formation = [target_formation[change_ids[i]] for i in change_ids]

# 绘制队形
import matplotlib.pyplot as plt
# 定义起始点和终止点的坐标
start_points = np.array(origin_formation)
end_points = np.array(real_target_formation)

# 创建三维画布
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制起始点和终止点
ax.scatter(start_points[:, 0], start_points[:, 1], start_points[:, 2], marker='o', color='blue', label='start')
ax.scatter(end_points[:, 0], end_points[:, 1], end_points[:, 2], marker='x', color='red', label='end')

# 绘制连接线
for i in range(len(start_points)):
    ax.plot([start_points[i, 0], end_points[i, 0]], [start_points[i, 1], end_points[i, 1]], [start_points[i, 2], end_points[i, 2]], color='gray', linestyle='--')

# 设置图例和标题
ax.legend()
ax.set_title('assignment')
ax.view_init(elev=90, azim=-90)
ax.set_xlabel('x')
ax.set_ylabel('y')
# 显示图形
plt.show()