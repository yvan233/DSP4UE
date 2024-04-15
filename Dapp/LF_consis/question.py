# import需求模块
import time
from DASP.module import Task, DaspCommon
from Agent.AirSimUavAgent import AirSimUavAgent
import pulp as pl
from scipy.optimize import linear_sum_assignment
from Dapp.LF_consis.formation_dict import formation_dict_9
import airsim
import numpy as np
import random
import copy
# 用户自定义函数区
Rate = 10
UE_ip = "127.0.0.1"
origin_geopoint = (116.16872381923261, 40.05405620434274,150)

def connect_airsim(name, origin_pos):
    # 连接AirSim模拟器
    uav = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= name, origin_pos=origin_pos)
    return uav

def follower_control(leader_state, follower_state, target_leader_state, target_follower_state, avoid_vel):
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

def follower_consensus_control(leaders_state, follower_state, leaders_target_state, target_follower_state, avoid_vel):
    Kp = 1.0
    Kp_avoid = 2.0 #避障系数
    v_max = 2.0 #最大速度
    vx = 0
    vy = 0
    vz = 0

    for leader_state, target_leader_state in zip(leaders_state, leaders_target_state):
        vx += Kp*(leader_state['position'][0] - follower_state['position'][0] - target_leader_state[0] + target_follower_state[0])
        vy += Kp*(leader_state['position'][1] - follower_state['position'][1] - target_leader_state[1] + target_follower_state[1])
        vz += Kp*(leader_state['position'][2] - follower_state['position'][2] - target_leader_state[2] + target_follower_state[2])
    vx = vx / len(leaders_state)
    vy = vy / len(leaders_state)
    vz = vz / len(leaders_state)
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
    avoid_radius = 2 # 避障半径
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

def avoid_control_node(uav_state):
    aid_vec1 = [1, 0, 0]
    aid_vec2 = [0, 1, 0]
    avoid_radius = 2 # 避障半径
    avoid_vel_node = np.array([0.0, 0.0, 0.0])  # 初始化避障控制量
    i = 0
    id = list(uav_state.keys())[0]
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
            avoid_vel_node += avoid_vel
    return avoid_vel_node

def get_cost_matrix(origin_formation, target_formation):
    origin_formation = np.array(origin_formation)
    origin_formation = (origin_formation - origin_formation[0, :])[1:]
    target_formation = np.array(target_formation)
    target_formation = (target_formation - target_formation[0, :])[1:]
    cost_matrix = np.linalg.norm(origin_formation[:, None] - target_formation, axis=2)
    return cost_matrix

    #匈牙利算法，以使得队形切换代价最小
def hungarian_algorithm(origin_formation, target_formation):
    # 计算两个队形的距离矩阵
    adj_matrix = get_cost_matrix(origin_formation, target_formation)
    row_index, col_index = linear_sum_assignment(adj_matrix)
    cost_sum = adj_matrix[row_index, col_index].sum()
    # new_formation = np.array(target_formation)[col_ind]
    return col_index + 1

    #混合整数线性规划,以使得队形切换时间最小
def milp_algorithm(origin_formation, target_formation):

    adj_matrix = get_cost_matrix(origin_formation, target_formation)
    prob = pl.LpProblem("example", pl.LpMinimize)
    size = range(adj_matrix.shape[0])
    # 创建变量
    t = pl.LpVariable("t", 0, None, pl.LpContinuous)
    c = pl.LpVariable.dicts("c", (size, size),0,1,pl.LpInteger)
    # 添加目标函数 最小化时间和一定程度上的代价
    prob += len(size)*t + pl.lpSum(adj_matrix[i][j]*c[i][j] for i in size for j in size)
    # 添加约束条件
    for i in size:
        for j in size:
            prob += t >= adj_matrix[i][j] * c[i][j]
    for i in size:
        prob += (pl.lpSum(c[i][j] for j in size) == 1)
    for j in size:
        prob += (pl.lpSum(c[i][j] for i in size) == 1)
    # 求解问题
    prob.solve()
    print(t.name,t.varValue)
    # 每行元素选择的列
    change_ids = [0 for i in size]
    for i in size:
        for j in size:
            if c[i][j].varValue == 1:
                change_ids[i] = j + 1
    return change_ids


#  
def taskFunction(self:Task, id, nbrDirection, datalist):
    uavid = int(id)-1
    name = f'Uav{uavid}'
    formation = formation_dict_9
    uav = connect_airsim(name, formation["origin"][uavid])
    uav.take_off(waited = True)
    
    Rate = 5
    sptIter = 0
    localLeaderNumMax = 3

    state = uav.get_state()
    self.sendDatatoGUI(state)
    location = DaspCommon.location
    location["X"] = state['position'][0]
    location["Y"] = state['position'][1]
    location["Z"] = state['position'][2]
    topology, nbrDistance = self.updateTopology(location)

    # nbrMessage = self.transmitData(nbrDirection,[state for _ in range(len(nbrDirection))])  
    # nbrDirection, nbrData = nbrMessage 

    origin_formation = formation["origin"]
    target_formation = formation["triangle"]
    print_iter = 0

    if uavid == 0:
        # 速度控制，会有静差
        # uav.move_by_velocity(1, 0, 0, duration = 100, yaw_mode=airsim.YawMode(True, 0))
        while True:
            last_time = time.time()
            state = uav.get_state()
            # nbrData.insert(0,state)
            # 获取所有无人机的状态
            # uav_state = {uav['id']:uav for uav in nbrData}
            # 计算所有无人机的避障速度
            # avoid_vel_dict = avoid_control(uav_state)
            # 广播到所有无人机
            Q = 0
            target_leader_state = target_formation[0]
            sendmessage = [Q, target_leader_state, state]
            nbrMessage = self.transmitData(nbrDirection,[sendmessage for _ in range(len(nbrDirection))])  
            nbrDirection, nbrData = nbrMessage               
            # uav.move_to_position(0, 0, -3, velocity=1)
            if print_iter % 10 == 0:
                print_iter = 0
                location = DaspCommon.location
                location["X"] = state['position'][0]
                location["Y"] = state['position'][1]
                location["Z"] = state['position'][2]
                topology, nbrDistance = self.updateTopology(location)
                nbrDirection = self.taskNbrDirection
                self.sendDatatoGUI(f"now_nbr_nodes:{topology}")
            print_iter += 1
            period = time.time() - last_time
            # 控制频率
            if period < 1/Rate:
                time.sleep(1/Rate - period) 

    else:
        
        # target_formation_index = hungarian_algorithm(origin_formation, target_formation)
        target_formation_index = milp_algorithm(origin_formation, target_formation)
        leader_id = int(self.leader)-1
        target_leader_state = target_formation[leader_id]
        target_follower_state = target_formation[target_formation_index[uavid-1]]  #这里默认了第一个是leader,修正下标
        Q = - np.linalg.norm(np.array(target_leader_state) - np.array(target_follower_state))
        self.sendDatatoGUI(f"target_follower_state:{target_follower_state}")

        while True:
            last_time = time.time()
            state = uav.get_state()
            sendmessage = [Q, target_follower_state, state]
            nbrMessage = self.transmitData(nbrDirection,[sendmessage for _ in range(len(nbrDirection))])  
            nbrDirection, nbrData = nbrMessage  
            # 
            # 取leader的数据
            # leader_state = nbrData[0][0]
            # avoid_vel_dict = nbrData[0][1]
            # avoid_vel = avoid_vel_dict[name]
            leaders_state = []
            leaders_target_state = []
            nbrQ = []
            nbrTargetState = []
            nbrTargetDis = []
            nbrState = []
            leaders_ids= []
            allState = []
            # 得到邻居数据
            for ele in nbrData:
                if ele:
                    nbrQ.append(ele[0])
                    nbrTargetState.append(ele[1])
                    nbrTargetDis.append(np.linalg.norm(np.array(ele[1]) - np.array(target_follower_state)))
                    nbrState.append(ele[2])
                else:
                    nbrQ.append(-2000)
                    nbrTargetState.append([0,0,0])
                    nbrTargetDis.append(2000)
                    nbrState.append(None)
            
            # 计算避障向量
            allState = copy.deepcopy(nbrState)
            allState.insert(0,state)
            allStateDict = {uav['id']:uav for uav in allState}
            # avoid_vel = avoid_control_node(allStateDict)
            # self.sendDatatoGUI(f"avoid_vel:{avoid_vel}")

            # 根据邻居距离自己的距离排序
            sorted_index = np.argsort(np.array(nbrTargetDis))
            # 选取距离最近的、Q值大于自己的至多localLeaderNumMax个邻居作为localLeader
            for i in range(len(nbrData)):
                if nbrQ[sorted_index[i]] > Q:
                    leaders_ids.append(nbrState[sorted_index[i]]['id'])
                    leaders_state.append(nbrState[sorted_index[i]])
                    leaders_target_state.append(nbrTargetState[sorted_index[i]])
                    if len(leaders_state) >= localLeaderNumMax:
                        break

            follower_state = state
            # 计算偏移量
            vx, vy, vz = follower_consensus_control(leaders_state, follower_state, leaders_target_state, target_follower_state, avoid_vel=[0,0,0])
            if print_iter % 10 == 0:
                # 每10次更新下拓扑
                print_iter = 0
                location = DaspCommon.location
                location["X"] = state['position'][0]
                location["Y"] = state['position'][1]
                location["Z"] = state['position'][2]
                topology, nbrDistance = self.updateTopology(location)
                nbrDirection = self.taskNbrDirection
                self.sendDatatoGUI(f"leaders_ids:{leaders_ids}, now_nbr_nodes:{topology}")
            print_iter += 1
            # 偏航角始终为0
            uav.move_by_velocity(vx, vy, vz, duration = 1, yaw_mode=airsim.YawMode(True, 0))
            period = time.time() - last_time
            # 控制频率
            if period < 1/Rate:
                time.sleep(1/Rate - period) 
            # # 将vx,vy,vz存储到csv文件中
            # import csv
            # with open(f'uav{uavid}_velocity.csv', 'a') as f:
            #     writer = csv.writer(f)
            #     writer.writerow([vx, vy, vz])

    self.sendDatatoGUI({name:uav.get_state()})
    return 0


