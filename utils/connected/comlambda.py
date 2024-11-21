import gurobipy as gp
from gurobipy import GRB
import numpy as np
import sys
sys.path.insert(1,".")
from Agent.formation.formation_dict import formation_dict_9, formation_dict_8, formation_dict_6
import random
import scipy.linalg as linalg

from Agent.formation.large_scale_formation.generate_tri import generate_triangle_points
from Agent.formation.large_scale_formation.generate_des_tri import generate_dense_triangle_points
from Agent.formation.large_scale_formation.generate_circle import generate_circle_points
from Agent.formation.large_scale_formation.generate_des_rec import generate_dense_rectangle_points

def algebraic_connectivity(adj_matrix):
    # 计算度矩阵 D
    degree_matrix = np.diag(np.sum(adj_matrix, axis=1))
    
    # 计算拉普拉斯矩阵 L = D - A
    laplacian_matrix = degree_matrix - adj_matrix
    
    # 计算拉普拉斯矩阵的特征值
    eigvals = linalg.eigvals(laplacian_matrix)
    
    # 排序特征值并返回第二小的特征值
    eigvals = np.sort(np.real(eigvals))  # 只取实部，因为特征值应该是实数
    return eigvals[1]  # 第二小的特征值

# 导入队形
# formation = formation_dict_9
# origin_formation = formation["origin"]
# target_formation = formation["triangle"]

# 生成指定队形
# origin_formation = generate_circle_points(leader_point = [0,0,0], num_points = 11, direction= [1,0,0], spacing= 3)
# target_formation = generate_triangle_points(leader_point = [0,0,0], num_points = 100, direction= [1,0,0], spacing= 3)


# 不同队形切换
# formation = generate_triangle_points(leader_point = [0,0,0], num_points = 100, direction= [0,1,0], spacing= 3)

formation =  generate_dense_triangle_points(leader_point = [0,0,0], num_points = 100, direction= [0,1,0], spacing= 3)


# formation  = generate_dense_rectangle_points(leader_point = [0,0,0], num_points = 100, direction= [0,1,0], spacing= 3)

adj_matrix = np.linalg.norm(np.array(formation)[:, None] - np.array(formation), axis=2)

CR = 4
adj_matrix = np.where(adj_matrix <= CR, 1, 0)
# 对角化置零
np.fill_diagonal(adj_matrix, 0)

eig = algebraic_connectivity(adj_matrix)
print("algebraic connectivity:", eig)

start_points = np.array(formation)

# 绘制队形
import matplotlib.pyplot as plt
# 创建三维画布
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111)

# 绘制起始点和终止点
ax.scatter(start_points[:, 0], start_points[:, 1], marker='o', color='blue', label='start')
plt.show()

