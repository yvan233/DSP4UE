import pandas as pd
import matplotlib.pyplot as plt

dir = "1111-5"
# file_names = [f'C:/Users/Yvan/Desktop/Java/Jbotsim/Node_{i}_data.csv' for i in range(100)]
file_names = [f'C:/Users/Yvan/Desktop/Java/Jbotsim/log/{dir}/Node_{i}_data.csv' for i in range(100)]
# file_names = [f'C:/Users/Yvan/Desktop/Java/Jbotsim/log/circle/Node_{i}_data.csv' for i in range(20)]
data_node_0 = pd.read_csv(f'C:/Users/Yvan/Desktop/Java/Jbotsim/log/{dir}/Node_0_data.csv')
# 创建一个图形和三个轴对象（X, Y, Z）
fig, ax = plt.subplots()
# 逐个文件加载数据并绘制曲线
for file_name in file_names:
    data = pd.read_csv(file_name)
    node_label = f'Node {file_name.split("_")[1]}'
    # 在三个不同的子图中绘制X, Y, Z与时间的关系
    # ax.plot(data['Time'], (data['VelocityX'] - data_node_0['VelocityX'])/ 10, label=node_label, lw=1)
    ax.plot(data['Time'], (data['AccelerationX'])/ 10, label=node_label, lw=1)
    # ax.plot(data['Time'], (data['PositionX']-data_node_0['PositionX'])/ 10, label=node_label, lw=1)
    # axs[1].plot(data['Time'], data['PositionY'] / 10, label=node_label)
    # axs[2].plot(data['Time'], data['PositionZ'] / 10, label=node_label)

# 设置每个轴的标签和标题
ax.set_ylabel('Position X (m)')
ax.set_title('Position X vs. Time')
# axs[1].set_ylabel('Position Y (m)')
# axs[1].set_title('Position Y vs. Time')
# axs[2].set_ylabel('Position Z (m)')
# axs[2].set_title('Position Z vs. Time')
# axs[2].set_xlabel('Time (s)')

# 为每个子图添加图例

# ax.legend()

# 调整子图的布局
plt.tight_layout()

# 显示图形
plt.show()

# plt.savefig(f'C:/Users/Yvan/Desktop/python/DSP4UE/debug/show/{dir}.png')