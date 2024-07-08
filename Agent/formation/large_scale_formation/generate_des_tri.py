import numpy as np
import matplotlib.pyplot as plt
import math
def generate_dense_triangle_points(leader_point = [0,0,0], num_points = 3, direction = [1,0,0], spacing = 3, theta = 60):
    """
    生成三角形的平面坐标。

    Args:
        leader_point: leader节点坐标
        direction: 前进方向
        spacing: 点与点之间的间距
        num_points: 点的数量。
        theta: 底边角度
    Returns:
        points: 所有节点平面坐标。
    """
    leader_point = np.array(leader_point)
    unit_vector = np.array(direction) / np.linalg.norm(direction)
    zvec = np.array([0,0,1])
    compvec = np.cross(unit_vector, zvec)

    midpoint = leader_point - unit_vector * spacing * np.sin(theta*np.pi/180)

    b = midpoint + spacing * np.cos(theta*np.pi/180) * compvec
    c = midpoint - spacing * np.cos(theta*np.pi/180) * compvec
    points = [leader_point, b, c]
    unit_b = b - leader_point
    unit_c = c - leader_point

    x = 3
    current_num = 3
    # 从第三层开始迭代
    while True:
        temp_b = leader_point + (x-1)*unit_b
        temp_c = leader_point + (x-1)*unit_c
        if num_points >= current_num + x:
            points.append(temp_b)
            points.append(temp_c)
            for i in range(1, x-1, 1):
                point = (i * temp_b + (x-1-i) * temp_c ) / (x-1)
                points.append(point)
            current_num += x
            x += 1
            
        else:
            end_num = num_points - current_num

            if end_num == 0:
                pass
            elif end_num == 1:
                #中点
                point = (temp_b + temp_c ) / 2
                points.append(point)
            elif end_num >=2:
                points.append(temp_b)
                points.append(temp_c)
                for i in range(1, end_num-1, 1):
                    point = (i * temp_b + (end_num-1-i) * temp_c ) / (end_num-1)
                    points.append(point)
            break
    return points

if __name__ == "__main__":
    points = generate_dense_triangle_points(leader_point = [0,0,0], num_points = 50, spacing= 3)
    plot_points = np.array(points)

    # 创建三维画布
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制起始点和终止点
    ax.scatter(plot_points[0, 0], plot_points[0, 1], plot_points[0, 2], marker='o', color='red', label='leader')

    ax.scatter(plot_points[1:, 0], plot_points[1:, 1], plot_points[1:, 2], marker='o', color='blue', label='follower')

    # 设置图例和标题
    ax.legend()
    ax.set_title('assignment')
    ax.set_xlim(-40,0)
    ax.set_ylim(-20,20)
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    # 显示图形
    plt.show()