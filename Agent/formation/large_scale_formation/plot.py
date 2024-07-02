import numpy as np
import matplotlib.pyplot as plt
# 定义起始点和终止点的坐标
start_points = np.array([[0, 0, 0], [10, 0, 0], [5, 8, 0]])
end_points = np.array([[5, 5, 5], [2, 2, 2], [8, 8, 8]])

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
ax.set_title('起始点和终止点连接图（三维）')

# 显示图形
plt.show()