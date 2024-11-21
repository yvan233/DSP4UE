import numpy as np
import matplotlib.pyplot as plt

def generate_hexagonal_layout(n):
    # 每个六边形之间的距离
    d = 2  # 每个节点的中心间距
    
    # 计算六边形的行数和列数
    rows = int(np.ceil(np.sqrt(n)))
    cols = rows
    
    # 存储所有节点的坐标
    positions = []
    
    # 计算每个节点的位置
    for i in range(rows):
        for j in range(cols):
            # 六边形排列的偏移量
            x_offset = j * 1.5 * d
            y_offset = i * np.sqrt(3) * d
            
            # 偶数行偏移量调整
            if j % 2 == 1:
                y_offset += np.sqrt(3) * d / 2
            
            positions.append((x_offset, y_offset))
    
    # 只返回前n个节点的位置
    return positions[:n]

# 输入节点数量
n = int(input("请输入节点数量 n: "))

# 生成六边形布局
positions = generate_hexagonal_layout(n)

# 可视化节点位置
x_vals, y_vals = zip(*positions)
plt.scatter(x_vals, y_vals, marker='o', color='blue')
plt.gca().set_aspect('equal', adjustable='box')
plt.title(f"Hexagonal Layout with {n} Nodes")
plt.show()
