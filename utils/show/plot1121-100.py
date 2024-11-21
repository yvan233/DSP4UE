# 绘制多条轨迹，并插空输出点的坐标，并绘制节点的连接关系
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.cm import get_cmap
from scipy.spatial.distance import cdist  # 用于计算节点之间的距离

# 文件夹路径
dir = "1121-100"
file_names = [f'C:/Users/Yvan/Desktop/Java/Jbotsim/log/{dir}/Node_{i}_data.csv' for i in range(100)]

# 创建图形和轴对象
fig, ax = plt.subplots(figsize=(12, 8))

# 获取颜色映射（为不同曲线分配不同颜色）
cmap = get_cmap('tab20')
num_files = len(file_names)
# colors = [cmap(i % 1) for i in range(num_files)]  # 循环使用颜色

# 颜色前10个为第一个颜色，每20个换个颜色
colors = [cmap(i // 10) for i in range(num_files)]  # 循环使用颜色

# colors = [cmap(i % 10) for i in range(num_files)]  # 循环使用颜色

# 交换0和5的颜色
colors[0], colors[4] = colors[4], colors[0]
# 手动选择点的索引
marker_indices = [0,  1250]

# 用于存储每个时刻的所有节点坐标
all_positions = {idx: [] for idx in marker_indices}

# 遍历所有节点的文件
for idx, file_name in enumerate(file_names):
    try:
        # 读取每个节点的数据
        data = pd.read_csv(file_name)
        data = data.iloc[:1501]

        # 提取轨迹点
        x = data['PositionX'].values / 10
        y = data['PositionY'].values / 10

        # 计算速度大小
        velocity_x = data['VelocityX'].values / 10
        velocity_y = data['VelocityY'].values / 10
        velocity = np.sqrt(velocity_x**2 + velocity_y**2)

        # 分段生成线段
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        # 创建线段集合并使用颜色映射表示速度
        lc = LineCollection(segments, cmap='autumn_r', norm=plt.Normalize(0, 4))
        lc.set_array(velocity)  # 使用速度数组来映射颜色
        lc.set_linewidth(0.5)

        # 添加到图像
        ax.add_collection(lc)

        # 绘制标记点
        valid_indices = [i for i in marker_indices if i < len(x)]
        ax.scatter(x[valid_indices], y[valid_indices], color=colors[idx], label=f'Node {idx}', s=50, zorder=5)

        # 存储这些点的位置用于连接关系
        for marker_index in valid_indices:
            all_positions[marker_index].append((x[marker_index], y[marker_index]))

    except Exception as e:
        print(f"Error processing file {file_name}: {e}")

# 绘制连接关系
threshold_distance = 12  # 设置距离阈值
for time_step, positions in all_positions.items():
    positions = np.array(positions)
    if len(positions) > 1:  # 至少两个点才能计算距离
        distances = cdist(positions, positions)  # 计算点之间的距离
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                if distances[i, j] < threshold_distance:
                    # 绘制连接线
                    ax.plot(
                        [positions[i, 0], positions[j, 0]],
                        [positions[i, 1], positions[j, 1]],
                        color='gray',
                        linestyle='-',
                        linewidth=0.5,
                        zorder=3,
                    )

# 设置图形属性
ax.set_xlabel('Position X (m)', fontsize=12)
ax.set_ylabel('Position Y (m)', fontsize=12)
ax.set_title('Drone Trajectories with Velocity and Connections', fontsize=14)
ax.axis('equal')  # 保持比例一致

# 添加颜色条
cbar = plt.colorbar(lc, ax=ax)
cbar.set_label('Velocity (m/s)', fontsize=12)

# 显示图例
# ax.legend(fontsize=10)

# 调整布局并显示
plt.tight_layout()
plt.show()
