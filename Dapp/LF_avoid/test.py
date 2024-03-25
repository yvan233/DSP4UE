import numpy as np
uav_state = {'Uav0': {'id': 'Uav0', 'timestamp': '1711359246873540200', 'position': [0.0, 0.0, -4.621467], 'orientation': [-0.0, 0.0, 1e-06], 'linear_velocity': [-2e-06, -1e-06, 0.008597], 'linear_acceleration': [3e-06, 4e-06, -0.076122], 'angular_velocity': [-0.0, 0.0, 0.0], 'angular_acceleration': [-0.0, -0.0, 0.0]}, 
             'Uav1': {'id': 'Uav1', 'timestamp': '1711359246670878000', 'position': [0.5, 0.5, -4.627668], 'orientation': [-0.0, -0.0, -1.2e-05], 'linear_velocity': [0.0, 0.0, 0.031104], 'linear_acceleration': [0.0, -0.0, -0.133497], 'angular_velocity': [-0.0, -0.0, 0.0], 'angular_acceleration': [-1.3e-05, -4e-06, -1e-06]}, 
             'Uav2': {'id': 'Uav2', 'timestamp': '1711359246670889000', 'position': [1, .0, -4.6], 'orientation': [0.0, 0.0, -6e-06], 'linear_velocity': [0.0, 0.0, 0.0], 'linear_acceleration': [0.0, 0.0, 0.0], 'angular_velocity': [0.0, 0.0, 0.0], 'angular_acceleration': [0.0, 0.0, 0.0]}, 'Uav3': {'id': 'Uav3', 'timestamp': '1711359246671003200', 'position': [-3.0, 2e-06, -4.62533], 'orientation': [0.0, 0.0, -1.8e-05], 'linear_velocity': [0.0, 0.0, 0.031133], 'linear_acceleration': [-0.0, 0.0, -0.133573], 'angular_velocity': [0.0, 0.0, 0.0], 'angular_acceleration': [-0.0, 0.0, -0.0]}, 'Uav4': {'id': 'Uav4', 'timestamp': '1711359246670918700', 'position': [-3.000001, 2.999999, -9.234745], 'orientation': [0.0, 0.0, -7e-06], 'linear_velocity': [-0.0, -0.0, 0.031201], 'linear_acceleration': [-0.0, 0.0, -0.133882], 'angular_velocity': [-0.0, -0.0, 0.0], 'angular_acceleration': [-9e-06, -7e-06, -0.0]}, 'Uav5': {'id': 'Uav5', 'timestamp': '1711359246670942300', 'position': [-2.999999, 6.0, 1.50455], 'orientation': [0.0, 0.0, 1e-06], 'linear_velocity': [0.0, 0.0, 0.0], 'linear_acceleration': [0.0, 0.0, 0.0], 'angular_velocity': [0.0, 0.0, 0.0], 'angular_acceleration': [0.0, 0.0, 0.0]}}
def avoid_control(uav_state):
    aid_vec1 = [1, 0, 0]
    aid_vec2 = [0, 1, 0]
    avoid_radius = 1.5  # 避障半径

    avoid_control_dict = {id: np.array([0.0, 0.0, 0.0]) for id in uav_state}  # 初始化避障控制量
    #枚举所有无人机
    for i,id in enumerate(uav_state):
        position_1 = np.array(uav_state[id]['position'])
        for j,nbr_id in enumerate(uav_state):
            if j<=i:
                continue
            position_2 = np.array(uav_state[nbr_id]['position'])
            dir_vec = position_1-position_2
            k = 1- np.linalg.norm(dir_vec) / avoid_radius
            if k > 0:
                cos1 = dir_vec.dot(aid_vec1)/(np.linalg.norm(dir_vec) * np.linalg.norm(aid_vec1))
                cos2 = dir_vec.dot(aid_vec2)/(np.linalg.norm(dir_vec) * np.linalg.norm(aid_vec2))
                if  abs(cos1) < abs(cos2):
                    avoid_control = k * np.cross(dir_vec, aid_vec1)/np.linalg.norm(np.cross(dir_vec, aid_vec1))
                else:
                    avoid_control = k * np.cross(dir_vec, aid_vec2)/np.linalg.norm(np.cross(dir_vec, aid_vec2))
                avoid_control_dict[id] += avoid_control
                avoid_control_dict[nbr_id] -= avoid_control
    avoid_control_dict = {key: avoid_control_dict[key].tolist() for key in avoid_control_dict}
    return avoid_control_dict
print(avoid_control(uav_state))