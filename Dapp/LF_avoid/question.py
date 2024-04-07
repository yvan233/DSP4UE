# import需求模块
import time
from DASP.module import Task
from Agent.AirSimUavAgent import AirSimUavAgent
import airsim
import numpy as np
# 用户自定义函数区
Rate = 10
UE_ip = "127.0.0.1"
origin_geopoint = (116.16872381923261, 40.05405620434274,150)

def connect_airsim(name, origin_pos):
    # 连接AirSim模拟器
    uav = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= name, origin_pos=origin_pos)
    return uav

def follower_control(uav, leader_state, follower_state, target_leader_state, target_follower_state, avoid_vel):
    Kp = 1.0
    Kp_avoid = 3.0 #避障系数
    v_max = 2.0 #最大速度

    vx = Kp*(leader_state['position'][0] - follower_state['position'][0] - target_leader_state[0] + target_follower_state[0])
    vy = Kp*(leader_state['position'][1] - follower_state['position'][1] - target_leader_state[1] + target_follower_state[1])
    vz = Kp*(leader_state['position'][2] - follower_state['position'][2] - target_leader_state[2] + target_follower_state[2])
    vx += Kp_avoid * avoid_vel[0]
    vy += Kp_avoid * avoid_vel[1]
    vz += Kp_avoid * avoid_vel[2]
    v_mag = (vx**2+vy**2+vz**2)**0.5
    if v_mag > v_max:
        vx = vx / v_mag * v_max
        vy = vy / v_mag * v_max
        vz = vz / v_mag * v_max        
    return vx, vy, vz

def avoid_control(uav_state):
    aid_vec1 = [1, 0, 0]
    aid_vec2 = [0, 1, 0]
    avoid_radius = 3 # 避障半径
    avoid_vel_dict = {id: np.array([0.0, 0.0, 0.0]) for id in uav_state}  # 初始化避障控制量
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
                    avoid_vel = k**0.5 * np.cross(dir_vec, aid_vec1)/np.linalg.norm(np.cross(dir_vec, aid_vec1))  #相比原文，加了个根号，在避障时更加平滑
                else:
                    avoid_vel = k**0.5 * np.cross(dir_vec, aid_vec2)/np.linalg.norm(np.cross(dir_vec, aid_vec2))
                avoid_vel_dict[id] += avoid_vel
                avoid_vel_dict[nbr_id] -= avoid_vel
    avoid_vel_dict = {key: avoid_vel_dict[key].tolist() for key in avoid_vel_dict}
    return avoid_vel_dict
    # pass

# 定义算法主函数taskFunction(self,id,nbrDirection,datalist)
# 四个形参分别为节点类，节点ID，节点邻居对应的方向以及初始化时键入的数据
def taskFunction(self:Task, id, nbrDirection, datalist):
    uavid = int(id)-1
    name = f'Uav{uavid}'
    formation = datalist[0]
    uav = connect_airsim(name, formation["origin"][uavid])
    uav.take_off(waited = True)


    state = uav.get_state()
    nbrMessage = self.transmitData(nbrDirection,[state for _ in range(len(nbrDirection))])  
    nbrDirection, nbrData = nbrMessage 
    if uavid == 0:
        # 速度控制，会有静差
        # uav.move_by_velocity(1, 0, 0, duration = 100, yaw_mode=airsim.YawMode(True, 0))
        while True:
            last_time = time.time()
            state = uav.get_state()
            nbrData.insert(0,state)
            # 获取所有无人机的状态
            uav_state = {uav['id']:uav for uav in nbrData}
            # 计算所有无人机的避障速度
            avoid_vel_dict = avoid_control(uav_state)
            # 广播到所有无人机
            sendmessage = [state, avoid_vel_dict]
            nbrMessage = self.transmitData(nbrDirection,[sendmessage for _ in range(len(nbrDirection))])  
            nbrDirection, nbrData = nbrMessage               
            # uav.move_to_position(0, 0, -3, velocity=1)
            period = time.time() - last_time
            # 控制频率
            if period < 1/Rate:
                time.sleep(1/Rate - period) 

    else:
        print_iter = 0
        while True:
            state = uav.get_state()
            nbrMessage = self.transmitData(nbrDirection,[state for _ in range(len(nbrDirection))])  
            nbrDirection, nbrData = nbrMessage  
            leader_id = int(self.leader)-1
            target_formation = formation["diamond"]
            target_leader_state = target_formation[leader_id]
            target_follower_state = target_formation[uavid]
            # 取leader的数据
            leader_state = nbrData[0][0]
            avoid_vel_dict = nbrData[0][1]
            avoid_vel = avoid_vel_dict[name]
            follower_state = state
            # 计算偏移量
            vx, vy, vz = follower_control(uav, leader_state, follower_state, target_leader_state, target_follower_state, avoid_vel)
            print_iter += 1
            if print_iter % 10 == 0:
                print_iter = 0
                self.sendDatatoGUI([avoid_vel,[vx, vy, vz]])
            uav.move_by_velocity(vx, vy, vz, duration = 1, yaw_mode=airsim.YawMode(True, 0))
            time.sleep(1/Rate)

    self.sendDatatoGUI({name:uav.get_state()})
    return 0


