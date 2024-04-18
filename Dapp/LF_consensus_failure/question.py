# import需求模块
import time
from DASP.module import Task, DaspCommon
from Agent.AirSimUavAgent import AirSimUavAgent
import pulp as pl
from scipy.optimize import linear_sum_assignment
from Agent.formation.formation_dict import formation_dict_9, formation_dict_8
import airsim
import numpy as np
import random
import copy
import csv
# 用户自定义函数区
Rate = 10
topoMaintTime = 1
SPTreeTime= 5
ExpendedTime = 3  #编队完成后持续时间
UE_ip = "127.0.0.1"
origin_geopoint = (116.16872381923261, 40.05405620434274,150)

def connect_airsim(name, origin_pos):
    # 连接AirSim模拟器
    uav = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= name, origin_pos=origin_pos)
    return uav

def follower_control(leader_state, follower_state, target_leader_state, target_follower_state, avoid_vel):
    # folloer速度跟随控制
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

def follower_control_acc(leader_state, follower_state, target_leader_state, target_follower_state, avoid_accel):
    # folloer加速度跟随控制
    Kp = 1.0
    gama = (4.0 / Kp) ** 0.5
    Kp_avoid = 2.0 #避障系数
    a_max = 2.0 #最大加速度速度
    
    ax = Kp*(leader_state['position'][0] - follower_state['position'][0] - target_leader_state[0] + target_follower_state[0]) \
        + gama * (leader_state['linear_velocity'][0] - follower_state['linear_velocity'][0])    
    ay = Kp*(leader_state['position'][1] - follower_state['position'][1] - target_leader_state[1] + target_follower_state[1]) \
        + gama * (leader_state['linear_velocity'][1] - follower_state['linear_velocity'][1])
    az = Kp*(leader_state['position'][2] - follower_state['position'][2] - target_leader_state[2] + target_follower_state[2]) \
        + gama * (leader_state['linear_velocity'][2] - follower_state['linear_velocity'][2])
    ax += Kp_avoid * avoid_accel[0]
    ay += Kp_avoid * avoid_accel[1]
    az += Kp_avoid * avoid_accel[2]
    a_mag = (ax**2+ay**2+az**2)**0.5
    if a_mag > a_max:
        ax = ax / a_mag * a_max
        ay = ay / a_mag * a_max
        az = az / a_mag * a_max        
    return ax, ay, az

def follower_consensus_control_acc(leaders_state, follower_state, leaders_target_state, target_follower_state, avoid_accel):
    # folloer共识算法加速度跟随控制
    Kp = 1.0
    gama = (4.0 / Kp) ** 0.5
    Kp_avoid = 3 #避障系数
    a_max = 2.0 #最大速度
    ax = 0
    ay = 0
    az = 0

    for leader_state, target_leader_state in zip(leaders_state, leaders_target_state):
        ax += Kp*(leader_state['position'][0] - follower_state['position'][0] - target_leader_state[0] + target_follower_state[0]) \
            + gama * (leader_state['linear_velocity'][0] - follower_state['linear_velocity'][0])    
        ay += Kp*(leader_state['position'][1] - follower_state['position'][1] - target_leader_state[1] + target_follower_state[1]) \
            + gama * (leader_state['linear_velocity'][1] - follower_state['linear_velocity'][1])   
        az += Kp*(leader_state['position'][2] - follower_state['position'][2] - target_leader_state[2] + target_follower_state[2]) \
            + gama * (leader_state['linear_velocity'][2] - follower_state['linear_velocity'][2])   
    ax = ax / len(leaders_state)
    ay = ay / len(leaders_state)
    az = az / len(leaders_state)
    ax += Kp_avoid * avoid_accel[0]
    ay += Kp_avoid * avoid_accel[1]
    az += Kp_avoid * avoid_accel[2]
    v_mag = (ax**2+ay**2+az**2)**0.5
    if v_mag > a_max:
        ax = ax / v_mag * a_max
        ay = ay / v_mag * a_max
        az = az / v_mag * a_max        
    return ax, ay, az

def follower_consensus_control(leaders_state, follower_state, leaders_target_state, target_follower_state, avoid_vel):
    # folloer共识算法速度跟随控制
    Kp = 1.0
    Kp_avoid = 3 #避障系数
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
    # 计算所有节点的避障向量
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
    # 计算本节点的避障向量
    aid_vec1 = [1, 0, 0]
    aid_vec2 = [0, 1, 0]
    avoid_radius = 2.5 # 避障半径
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

    return avoid_vel_node.tolist()

def get_cost_matrix(origin_formation, target_formation):
    # 获取两个队形的距离矩阵
    origin_formation = np.array(origin_formation)
    origin_formation = (origin_formation - origin_formation[0, :])[1:]
    target_formation = np.array(target_formation)
    target_formation = (target_formation - target_formation[0, :])[1:]
    cost_matrix = np.linalg.norm(origin_formation[:, None] - target_formation, axis=2)
    return cost_matrix

    
def hungarian_algorithm(origin_formation, target_formation):
    #匈牙利算法，以使得队形切换代价最小
    adj_matrix = get_cost_matrix(origin_formation, target_formation)
    row_index, col_index = linear_sum_assignment(adj_matrix)
    cost_sum = adj_matrix[row_index, col_index].sum()
    # new_formation = np.array(target_formation)[col_ind]
    return col_index + 1

def milp_algorithm(origin_formation, target_formation, origin_ids):
    #混合整数线性规划,以使得队形切换时间最小 origin_ids为原始队形的id
    # adj_matrix = get_cost_matrix(origin_formation, target_formation)
    adj_matrix = np.linalg.norm(np.array(origin_formation)[:, None] - np.array(target_formation), axis=2)
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
    change_ids = {}
    for i in size:
        for j in size:
            if c[i][j].varValue == 1:
                change_ids[origin_ids[i]] = j
    return change_ids

def judge_finish(leaders_state, follower_state, leaders_target_state, target_follower_state):
    dx = 0
    dy = 0
    dz = 0
    threshold = 0.01
    z_threshold = 0.1

    for leader_state, target_leader_state in zip(leaders_state, leaders_target_state):
        dx += (leader_state['position'][0] - follower_state['position'][0] - target_leader_state[0] + target_follower_state[0]) ** 2\
            +  (leader_state['linear_velocity'][0] - follower_state['linear_velocity'][0]) ** 2
        dy += (leader_state['position'][1] - follower_state['position'][1] - target_leader_state[1] + target_follower_state[1]) ** 2\
            +  (leader_state['linear_velocity'][1] - follower_state['linear_velocity'][1]) ** 2
        dz += (leader_state['position'][2] - follower_state['position'][2] - target_leader_state[2] + target_follower_state[2]) ** 2\
            +  (leader_state['linear_velocity'][2] - follower_state['linear_velocity'][2]) ** 2
    dx = dx / len(leaders_state)
    dy = dy / len(leaders_state)
    dz = dz / len(leaders_state)
    if dx < threshold and dy < threshold and dz < z_threshold:
        return 1
    else:
        return 0

def consensus_formation(self:Task, uavid, uav:AirSimUavAgent, nbrDirection, origin_formation, target_formation, origin_ids):
    localLeaderNumMax = 3 # 节点跟踪的邻居数量
    location = DaspCommon.location
    print_iter = 0  # 打印迭代器
    finishFlag = 0  # 编队完成标志，1表示完成，0表示未完成
    finishRound = -1 # 编队算法结束轮次

    state = uav.get_state()
    location["X"] = state['position'][0]
    location["Y"] = state['position'][1]
    location["Z"] = state['position'][2]
    topology, nbrDistance = self.updateTopology(location)
    nbrDirection = self.taskNbrDirection

    target_formation_index = milp_algorithm(origin_formation, target_formation, origin_ids)
    real_target_formation = [target_formation[target_formation_index[i]] for i in target_formation_index]
    if target_formation_index[uavid] == 0:
        uav.move_by_velocity(1, 0, 0, duration = 100, yaw_mode=airsim.YawMode(True, 0))
        Q = 0
        target_leader_state = target_formation[0]
        tree = self.updateSpanningTree(minValue = -Q)
        while True:
            if finishRound == 0:
                break
            last_time = time.time()
            state = uav.get_state()

            # 传给邻居的信息包括自身的q值，目标队形的自身位置，当前自身状态
            sendmessage = [Q, target_leader_state, state, finishFlag, finishRound] 
            nbrMessage = self.transmitData(nbrDirection,[sendmessage for _ in range(len(nbrDirection))])  
            nbrDirection, nbrData = nbrMessage     

            # 邻居结束标志
            nbrFinishFlagList = [ele[3] for ele in nbrData]
            # 只有当自己的子节点都完成编队时，自己才能完成编队
            finishFlag = 1
            if not self.childDirection:
                pass
            else:
                for direction,nbrFinishFlag in zip(nbrDirection,nbrFinishFlagList):
                    if direction in self.childDirection:
                        if nbrFinishFlag == 0:
                            finishFlag = 0
                            break
            
            # 当leader节点的finishFlag为1时，表示编队完成
            if finishFlag:
                if finishRound == -1:
                    finishRound = len(origin_formation) - 1 + ExpendedTime * Rate # 编队结束轮次为节点数
                else:
                    finishRound -= 1
            # 如果结束中节点故障了，那么leader节点重置结束轮次
            else:
                finishRound = -1

            if print_iter % (topoMaintTime * Rate) == 0:
                location["X"] = state['position'][0]
                location["Y"] = state['position'][1]
                location["Z"] = state['position'][2]
                topology, nbrDistance = self.updateTopology(location)
                nbrDirection = self.taskNbrDirection
                self.sendDatatoGUI(f"now_nbr_nodes:{topology}, finishFlag:{finishFlag}, finishRound:{finishRound}")
                if print_iter == SPTreeTime * Rate:
                    tree = self.updateSpanningTree(minValue = -Q)
                    self.sendDatatoGUI(tree)
                    print_iter = 0                
            print_iter += 1
            period = time.time() - last_time
            # 控制频率
            if period < 1/Rate:
                time.sleep(1/Rate - period) 
    else:
        target_leader_state = target_formation[0]
        target_follower_state = target_formation[target_formation_index[uavid]] 
        Q = - np.linalg.norm(np.array(target_leader_state) - np.array(target_follower_state))
        tree = self.updateSpanningTree(minValue = -Q)
        self.sendDatatoGUI(f"target_follower_state:{target_follower_state}")
        while True:
            if finishRound == 0:
                break
            last_time = time.time()
            state = uav.get_state()
            sendmessage = [Q, target_follower_state, state, finishFlag, finishRound] 
            nbrMessage = self.transmitData(nbrDirection,[sendmessage for _ in range(len(nbrDirection))])  
            nbrDirection, nbrData = nbrMessage  
            leaders_state = []
            leaders_target_state = []
            nbrQ = []
            nbrTargetState = []
            nbrTargetDis = []
            nbrState = []
            leaders_ids= []
            allState = []

            nbrFinishFlagList = [ele[3] for ele in nbrData]
            nbrFinieshRoundList = [ele[4] for ele in nbrData]
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
            avoid_acc = avoid_control_node(allStateDict)
            

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
            ax, ay, az = follower_consensus_control_acc(leaders_state, follower_state, leaders_target_state, target_follower_state, avoid_acc)
            
            # 判断编队是否完成
            finishFlag = judge_finish(leaders_state, follower_state, leaders_target_state, target_follower_state)

            # 只有当自己的子节点都完成编队时，自己才能完成编队
            if finishFlag:
                if not self.childDirection:
                    finishFlag = 1
                else:
                    for direction,nbrFinishFlag in zip(nbrDirection,nbrFinishFlagList):
                        if direction in self.childDirection:
                            if nbrFinishFlag == 0:
                                finishFlag = 0
                                break

            # 节点的结束轮次为leader结束轮次-1
            if finishFlag:
                if self.parentDirection in nbrDirection:
                    index = nbrDirection.index(self.parentDirection)
                    parentRound = nbrFinieshRoundList[index]
                    if parentRound != -1:
                        finishRound = parentRound - 1
                    else:
                        finishRound = -1
                else:
                    pass #等待生成树重置

            uav.move_by_acceleration(ax, ay, az, duration = 1)

            if print_iter % (topoMaintTime * Rate) == 0:
                # 每10次更新下拓扑
                self.sendDatatoGUI([avoid_acc,[ax, ay, az], finishFlag, finishRound])
                location["X"] = state['position'][0]
                location["Y"] = state['position'][1]
                location["Z"] = state['position'][2]
                topology, nbrDistance = self.updateTopology(location)
                # 更新邻居方向
                nbrDirection = self.taskNbrDirection
                self.sendDatatoGUI(f"leaders_ids:{leaders_ids}, now_nbr_nodes:{topology}, now nbrDirection:{nbrDirection}")
                if print_iter == SPTreeTime * Rate:
                    # 每50次更新下最小生成树
                    tree = self.updateSpanningTree(minValue = -Q)
                    self.sendDatatoGUI(tree)
                    print_iter = 0
                uav.move_by_acceleration(ax, ay, az, duration = 1)
            print_iter += 1            
            period = time.time() - last_time
            # 控制频率
            if period < 1/Rate:
                time.sleep(1/Rate - period) 
    return real_target_formation

def taskFunction(self:Task, id, nbrDirection, datalist):
    uavid = int(id)-1
    name = f'Uav{uavid}'
    formation = formation_dict_9
    formation_8 = formation_dict_8
    origin_formation = formation["origin"]
    target_formation = formation["triangle"]
    target_formation2 = formation["rectangle"]
    target_formation_81 = formation_8["triangle"]
    target_formation_82 = formation_8["rectangle"]
    uav = connect_airsim(name, origin_formation[uavid])
    uav.take_off(waited = True)

    origin_ids = [i for i in range(len(origin_formation))]
    last_formation = consensus_formation(self, uavid, uav, nbrDirection, origin_formation, target_formation, origin_ids)

    # last_formation = consensus_formation(self, uavid, uav, nbrDirection, last_formation, target_formation2)

    location = DaspCommon.location
    deleteUavId = 1

    if uavid == deleteUavId:
        uav.move_by_acceleration(0, 0, 10, duration = 2)

        print_iter = 0
        location["X"] = 1000
        location["Y"] = 1000
        location["Z"] = 1000
        topology, nbrDistance = self.updateTopology(location)
        nbrDirection = self.taskNbrDirection
        while True:
            last_time = time.time()
            if print_iter % (topoMaintTime * Rate) == 0:
                topology, nbrDistance = self.updateTopology(location)
                nbrDirection = self.taskNbrDirection
                self.sendDatatoGUI(f"now_nbr_nodes:{topology}")
            print_iter += 1
            period = time.time() - last_time
            # 控制频率
            if period < 1/Rate:
                time.sleep(1/Rate - period)
    else:
        uav.hover()
        topology, nbrDistance = self.updateTopology(location)
        nbrDirection = self.taskNbrDirection

        del origin_ids[deleteUavId]
        del last_formation[deleteUavId]
        last_formation = consensus_formation(self, uavid, uav, nbrDirection, last_formation, target_formation_81, origin_ids)

        uav.hover()
        


    return 0


