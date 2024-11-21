import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

# 文件夹路径
dir = "1121"
file_names = [f'C:/Users/Yvan/Desktop/Java/Jbotsim/log/{dir}/Node_{i}_data.csv' for i in range(20)]

# 创建图形和轴对象
fig, ax = plt.subplots(figsize=(12, 8))

# 遍历所有节点的文件
for file_name in file_names:
    try:
        # 读取每个节点的数据
        data = pd.read_csv(file_name)

        # 提取轨迹点
        x = data['PositionX'].values/10
        y = data['PositionY'].values/10

        # 计算速度大小
        velocity_x = data['VelocityX'].values/10
        velocity_y = data['VelocityY'].values/10
        velocity = np.sqrt(velocity_x**2 + velocity_y**2)

        # 分段生成线段
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        # 创建线段集合并使用颜色映射表示速度
        lc = LineCollection(segments, cmap='autumn_r', norm=plt.Normalize(0,5))

        lc.set_array(velocity)  # 使用速度数组来映射颜色
        lc.set_linewidth(2)

        # 添加到图像
        ax.add_collection(lc)

        # 可选：标注起点和终点
        ax.scatter(x[0], y[0], c='blue', label=f'{file_name.split("/")[-1]} Start', s=10)
        ax.scatter(x[-1], y[-1], c='red', label=f'{file_name.split("/")[-1]} End', s=10)

    except Exception as e:
        print(f"Error processing file {file_name}: {e}")

# 设置图形属性
ax.set_xlabel('Position X (m)', fontsize=12)
ax.set_ylabel('Position Y (m)', fontsize=12)
ax.set_title('Drone Trajectories with Velocity', fontsize=14)
ax.axis('equal')  # 保持比例一致
# ax.set_xlim(-10, 150)  # 根据数据调整坐标范围
# ax.set_ylim(-50, 50)  # 根据数据调整坐标范围

# 添加颜色条
cbar = plt.colorbar(lc, ax=ax)
cbar.set_label('Velocity (m/s)', fontsize=12)

# 调整布局并显示
plt.tight_layout()
plt.show()
