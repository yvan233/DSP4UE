import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from matplotlib.colors import Normalize

# 文件夹路径
dir = "1111-5"
file_names = [f'C:/Users/Yvan/Desktop/Java/Jbotsim/log/{dir}/Node_{i}_data.csv' for i in range(20)]

# 创建三维图形对象
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# 遍历所有节点的文件
for file_name in file_names:
    try:
        # 读取每个节点的数据
        data = pd.read_csv(file_name)

        # 提取轨迹点
        x = data['PositionX'].values
        y = data['PositionY'].values
        z = data['PositionZ'].values  # 添加Z轴位置

        # 计算速度大小
        velocity_x = data['VelocityX'].values
        velocity_y = data['VelocityY'].values
        velocity_z = data['VelocityZ'].values
        velocity = np.sqrt(velocity_x**2 + velocity_y**2 + velocity_z**2)

        # 分段生成线段
        points = np.array([x, y, z]).T.reshape(-1, 1, 3)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        # 创建三维线段集合并用颜色映射速度
        lc = Line3DCollection(segments, cmap='autumn_r', norm=Normalize(0, 5))
        lc.set_array(velocity)  # 使用速度数组来映射颜色
        lc.set_linewidth(2)

        # 添加到三维图像
        ax.add_collection3d(lc)

        # 可选：标注起点和终点
        ax.scatter(x[0], y[0], z[0], c='blue', label=f'{file_name.split("/")[-1]} Start', s=10)
        ax.scatter(x[-1], y[-1], z[-1], c='red', label=f'{file_name.split("/")[-1]} End', s=10)

    except Exception as e:
        print(f"Error processing file {file_name}: {e}")

# 设置三维图形属性
ax.set_xlabel('Position X (m)', fontsize=12)
ax.set_ylabel('Position Y (m)', fontsize=12)
ax.set_zlabel('Position Z (m)', fontsize=12)
ax.set_title('3D Drone Trajectories with Velocity', fontsize=14)

# 添加颜色条
sm = plt.cm.ScalarMappable(cmap='autumn_r', norm=Normalize(0, velocity.max()))
sm.set_array([])
cbar = fig.colorbar(sm, ax=ax, pad=0.1)
cbar.set_label('Velocity (m/s)', fontsize=12)

# 调整显示比例
ax.set_box_aspect([1, 1, 0.5])  # 设置 X, Y, Z 轴比例
plt.tight_layout()
plt.show()
