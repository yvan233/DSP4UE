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

def follower_control(uav, leader_state, follower_state, target_leader_state, target_follower_state):
    Kp = 1.0
    Kp_avoid = 2.0
    v_max = 2.0

    vx = Kp*(leader_state['position'][0] - follower_state['position'][0] - target_leader_state[0] + target_follower_state[0])
    vy = Kp*(leader_state['position'][1] - follower_state['position'][1] - target_leader_state[1] + target_follower_state[1])
    vz = Kp*(leader_state['position'][2] - follower_state['position'][2] - target_leader_state[2] + target_follower_state[2])
    v_mag = (vx**2+vy**2+vz**2)**0.5
    if v_mag > v_max:
        vx = vx / v_mag * v_max
        vy = vy / v_mag * v_max
        vz = vz / v_mag * v_max    
    
    return vx, vy, vz


# 定义算法主函数taskFunction(self,id,nbrDirection,datalist)
# 四个形参分别为节点类，节点ID，节点邻居对应的方向以及初始化时键入的数据
def taskFunction(self:Task, id, nbrDirection, datalist):
    uavid = int(id)-1
    name = f'Uav{uavid}'
    formation = datalist[0]
    uav = connect_airsim(name, formation["origin"][uavid])
    uav.take_off(waited = True)

    if uavid == 0:
        # 速度控制，会有静差
        # uav.move_by_velocity(1, 0, 0, duration = 100, yaw_mode=airsim.YawMode(True, 0))
        while True:
            state = uav.get_state()
            nbrData = self.transmitData(nbrDirection,[state for _ in range(len(nbrDirection))])  
            nbrDirection_fb, nbrData_fb = nbrData               
            # uav.move_to_position(0, 0, -3, velocity=1)
            # self.sendDatatoGUI(nbrData)
            time.sleep(1/Rate)

    else:
        while True:
            state = uav.get_state()
            nbrData = self.transmitData(nbrDirection,[state for _ in range(len(nbrDirection))])  
            nbrDirection_fb, nbrData_fb = nbrData 
            # 
            
            leader_id = int(self.leader)-1
            target_formation = formation["diamond"]
            target_leader_state = target_formation[leader_id]
            target_follower_state = target_formation[uavid]
            leader_state = nbrData_fb[0]
            follower_state = state
            # 计算偏移量
            vx, vy, vz = follower_control(uav, leader_state, follower_state, target_leader_state, target_follower_state)
            self.sendDatatoGUI([vx, vy, vz])
            uav.move_by_velocity(vx, vy, vz, duration = 1, yaw_mode=airsim.YawMode(True, 0))
            # time.sleep(1/Rate)

    self.sendDatatoGUI({name:uav.get_state()})
    return 0


