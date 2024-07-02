import numpy as np

def calculate_coordinates(vertex, direction, length):
    # 计算中点坐标
    midpoint = np.array(vertex) + 0.5 * np.array(direction)

    # 计算单位方向向量
    unit_vector = np.array(direction) / np.linalg.norm(direction)

    # 计算另两个顶点坐标
    point1 = midpoint + 0.5 * length * unit_vector
    point2 = midpoint - 0.5 * length * unit_vector

    return point1, point2

# 示例数据
vertex = [0, 0, 0]
direction = [1, 0]
length = 10

# 计算坐标
point1, point2 = calculate_coordinates(vertex, direction, length)

print("Point 1: ", point1)
print("Point 2: ", point2)