import numpy as np
import matplotlib.pyplot as plt
import math
def generate_dense_rectangle_points(leader_point = [0,0,0], num_points = 4, direction = [1,0,0], spacing = 3):
    """
    生成矩形的平面坐标。

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
    unit_vector = - np.array(direction) / np.linalg.norm(direction)
    zvec = np.array([0,0,1])
    unit_vector2 =  - np.cross(unit_vector, zvec)

    # 每层数量
    num_width = math.ceil(math.sqrt(num_points))
    # 层数
    num_height = num_points // num_width
    # 底层数量
    num_buttom = num_points % num_width

    points = []

    for i in range(num_height):
        for j in range(num_width):
            point = (i * spacing * unit_vector + j * spacing * unit_vector2) + leader_point
            points.append(point)

    temp_b = (num_height * spacing * unit_vector + 0 * spacing * unit_vector2) + leader_point
    temp_c = (num_height * spacing * unit_vector + (num_width-1) * spacing * unit_vector2) + leader_point

    if num_buttom == 0:
        pass
    elif num_buttom == 1:
        #中点
        point = (temp_b + temp_c ) / 2
        points.append(point)
    elif num_buttom >=2:
        points.append(temp_b)
        points.append(temp_c)
        for i in range(1, num_buttom-1, 1):
            point = (i * temp_b + (num_buttom-1-i) * temp_c ) / (num_buttom-1)
            points.append(point)

    # 修改leader节点为中点
    leader_id = math.ceil(num_width/2) - 1
    
    points[0],points[leader_id] = points[leader_id],points[0]

    points = [ele - points[0] + leader_point for ele in points]

    return points

if __name__ == "__main__":
    points = generate_dense_rectangle_points(leader_point = [0,0,0], num_points = 10, spacing= 3)
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
    # ax.set_xlim(-40,0)
    # ax.set_ylim(-20,20)
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    # 显示图形
    plt.show()