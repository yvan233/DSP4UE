import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from scipy.linalg import eigvals

# 文件路径和节点数量
dir = "1121-100-6"
file_names = [f'C:/Users/Yvan/Desktop/Java/Jbotsim/log/{dir}/Node_{i}_data.csv' for i in range(100)]

dt = 0.04  # 时间步长
# 选定的时刻（marker_indices）
marker_indices = [0, 1295, 2590,3750]
threshold_distance = 12  # 距离阈值用于构建邻接矩阵

# 初始化存储
times = None
velocities_x = []  # 存储 x 方向速度
velocities_y = []  # 存储 y 方向速度
positions = {key: [] for key in marker_indices}  # 存储每个时刻的节点位置
algebraic_connectivity = []  # 代数连通度结果

# 读取所有节点数据
for idx, file_name in enumerate(file_names):
    data = pd.read_csv(file_name)
    # data = data.iloc[:1600]  # 截断数据，保留前 1600 个时刻
    # times = data['Time'].values / 100  # 假设时间列为 'Time'
    times = np.arange(len(data)) * dt  # 计算实际时间序列

    # 提取速度分量
    velocity_x = data['VelocityX'].values / 10
    velocity_y = data['VelocityY'].values / 10

    # 存储速度分量
    velocities_x.append(velocity_x)
    velocities_y.append(velocity_y)

    # 提取位置信息
    x = data['PositionX'].values / 10
    y = data['PositionY'].values / 10

    # 保存指定时刻的节点位置
    for marker_index in marker_indices:
        if marker_index < len(x):
            positions[marker_index].append([x[marker_index], y[marker_index]])

# 计算代数连通度
for marker_index in marker_indices:
    pos = np.array(positions[marker_index])
    if len(pos) > 1:
        distances = cdist(pos, pos)  # 计算节点之间的距离
        adjacency_matrix = (distances < threshold_distance).astype(float)  # 邻接矩阵
        degree_matrix = np.diag(np.sum(adjacency_matrix, axis=1))  # 度矩阵
        laplacian_matrix = degree_matrix - adjacency_matrix  # 拉普拉斯矩阵
        eigenvalues = np.sort(np.real(eigvals(laplacian_matrix)))  # 特征值
        algebraic_connectivity.append(eigenvalues[1])  # 第二小的特征值为代数连通度
    else:
        algebraic_connectivity.append(0)  # 单节点无连通度

# 绘制图形
fig, axs = plt.subplots(3, 1, figsize=(9, 6))

# 绘制 x 方向速度
for velocity_x in velocities_x:
    axs[0].plot(times[:len(velocity_x)], velocity_x, alpha=0.7)
axs[0].set_title('X Velocity over Time')
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('X Velocity (m/s)')

# 绘制 y 方向速度
for velocity_y in velocities_y:
    axs[1].plot(times[:len(velocity_y)], velocity_y, alpha=0.7)
axs[1].set_title('Y Velocity over Time')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Y Velocity (m/s)')

# 绘制代数连通度随时间变化
marker_times = [i * dt for i in marker_indices]  # 将 marker_indices 转换为实际时间
axs[2].plot(marker_times, algebraic_connectivity, '-o', color='red')
axs[2].set_title('Algebraic Connectivity at Selected Time Steps')
axs[2].set_xlabel('Time (s)')
axs[2].set_ylabel('Algebraic Connectivity')

plt.tight_layout()
plt.savefig(f"utils\show\{dir}-velocity_and_connectivity.png", dpi=600, bbox_inches='tight')
plt.show()
