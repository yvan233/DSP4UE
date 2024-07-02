import numpy as np
import matplotlib.pyplot as plt
import math
def generate_triangle_points(leader_point = [0,0,0], num_points = 3, direction = [1,0,0], spacing = 3, theta = 60):
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
    # 每条边分配的节点数
    num_edge_points = math.ceil(num_points/3)

    # 底边节点数
    num_buttom_points = num_points - 2 * num_edge_points

    # 边长
    length = spacing * num_edge_points

    leader_point = np.array(leader_point)
    unit_vector = np.array(direction) / np.linalg.norm(direction)
    zvec = np.array([0,0,1])
    compvec = np.cross(unit_vector, zvec)


    midpoint = leader_point - unit_vector * length * np.sin(theta*np.pi/180)

    b = midpoint + length * np.cos(theta*np.pi/180) * compvec
    c = midpoint - length * np.cos(theta*np.pi/180) * compvec

    points = [leader_point]

    for i in range(1, num_edge_points + 1, 1):
        point1 = (i * b + (num_edge_points-i) * leader_point) / num_edge_points
        point2 = (i * c + (num_edge_points-i) * leader_point) / num_edge_points
        points.append(point1)
        points.append(point2)

    for i in range(1, num_buttom_points, 1):
        point = (i * b + (num_buttom_points-i) * c ) / num_buttom_points
        points.append(point)
    return points

if __name__ == "__main__":
    points = generate_triangle_points(leader_point = [0,0,0], num_points = 30, spacing= 3)
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
    # ax.set_xlim(-10,10)
    # ax.set_ylim(-10,10)
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    # 显示图形
    plt.show()