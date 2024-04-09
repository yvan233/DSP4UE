import pulp as pl
import numpy as np
# 多目标优化

# 创建代价举证
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
# cost_matrix = np.array([[3, 2, 3], [2, 0, 5], [3, 2, 2]])

# 创建问题实例
prob = pl.LpProblem("example", pl.LpMinimize)

size = range(cost_matrix.shape[0])
# 创建变量
t = pl.LpVariable("t", 0, None, pl.LpContinuous)
c = pl.LpVariable.dicts("c", (size, size),0,1,pl.LpInteger)

# 添加目标函数
# prob += pl.lpSum(cost_matrix[i][j] * c[i][j] for i in size for j in size)
prob += len(size)*t + pl.lpSum(cost_matrix[i][j]*c[i][j] for i in size for j in size)
# 添加约束条件

for i in size:
    for j in size:
        prob += t >= cost_matrix[i][j] * c[i][j]
for i in size:
    prob += (pl.lpSum(c[i][j] for j in size) == 1)
for j in size:
    prob += (pl.lpSum(c[i][j] for i in size) == 1)
# 求解问题
prob.solve()

# 输出结果
# for v in prob.variables():
#     print(v.name, "=", v.varValue)
print(t.name,t.varValue)

#输出代价之和
cost_sum = 0
for i in size:
    for j in size:
        cost_sum += cost_matrix[i][j] * c[i][j].varValue
print(cost_sum)

# 每行元素选择的列
change_ids = [i for i in size]
for i in size:
    for j in size:
        if c[i][j].varValue == 1:
            change_ids[i] = j
print(change_ids)