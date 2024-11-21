# 绘制多条轨迹，并插空输出点的坐标

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.cm import get_cmap

# 文件夹路径
dir = "1121"
file_names = [f'C:/Users/Yvan/Desktop/Java/Jbotsim/log/{dir}/Node_{i}_data.csv' for i in range(16)]

# 创建图形和轴对象
fig, ax = plt.subplots(figsize=(12, 8))

# 获取颜色映射（为不同曲线分配不同颜色）
cmap = get_cmap('tab20')  # 可以改成 'Set1' 或其他离散配色方案
num_files = len(file_names)
colors = [cmap(i % 20) for i in range(num_files)]  # 循环使用颜色


marker_indices = [0,400, 800, 1200,1500]  # 手动选择点的索引
# 遍历所有节点的文件
for idx, file_name in enumerate(file_names):
    try:
        # 读取每个节点的数据
        data = pd.read_csv(file_name)
        data = data.iloc[:1600]

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
        lc.set_linewidth(1)

        # 添加到图像
        ax.add_collection(lc)

        
        valid_indices = [i for i in marker_indices if i < len(x)]  # 确保索引在范围内
        ax.scatter(x[marker_indices], y[marker_indices], color=colors[idx], label=f'Node {idx}', s=50, zorder=5)

    except Exception as e:
        print(f"Error processing file {file_name}: {e}")

# 设置图形属性
ax.set_xlabel('Position X (m)', fontsize=12)
ax.set_ylabel('Position Y (m)', fontsize=12)
ax.set_title('Drone Trajectories with Velocity', fontsize=14)
ax.axis('equal')  # 保持比例一致

# 添加颜色条
cbar = plt.colorbar(lc, ax=ax)
cbar.set_label('Velocity (m/s)', fontsize=12)

# 显示图例
ax.legend(loc='upper right', fontsize=10)

# 调整布局并显示
plt.tight_layout()
plt.show()
