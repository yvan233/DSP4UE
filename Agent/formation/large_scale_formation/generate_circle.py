import numpy as np
import matplotlib.pyplot as plt
import math
def generate_circle_points(leader_point = [0,0,0], num_points = 3, direction = [1,0,0], spacing = 3):
    """
    生成圆形的平面坐标。

    Args:
        leader_point: leader节点坐标
        direction: 前进方向
        spacing: 点与点之间的间距
        num_points: 点的数量。
    Returns:
        points: 所有节点平面坐标。
    """ 

    # 圆心角
    theta = 2*np.pi / num_points
    r = spacing / 2 / np.sin(theta/2)
    leader_point = np.array(leader_point)
    unit_vector = np.array(direction) / np.linalg.norm(direction)
    O = leader_point - unit_vector * r

    # leader节点的角度
    alpha = np.arctan2(unit_vector[1],unit_vector[0])
    # 生成等距的圆心角
    theta_list = np.linspace(alpha, 2 * np.pi + alpha, num_points, endpoint=False)

    # 生成圆形坐标
    x = r * np.cos(theta_list) + O[0]
    y = r * np.sin(theta_list) + O[1]
    z = np.zeros_like(theta_list) + O[2]
    
    return np.stack([x, y, z], axis=1)

if __name__ == "__main__":
    points = generate_circle_points(leader_point = [0,0,0], num_points = 10, direction = [-1,1,0])
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