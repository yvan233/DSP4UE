import sys
sys.path.insert(1,".")  # 把上一级目录加入搜索路径
from Agent.AirSimCarAgent import AirSimCarAgent
from threading import Thread
import numpy as np
import time
import copy
import os

# 计算转角
def Propotional_navi(cur_obs, pre_obs, catcher_name, K) -> float: 
    cur_relative_vec = cur_obs["Escaper"] - cur_obs[catcher_name] 
    pre_relative_vec = pre_obs["Escaper"] - pre_obs[catcher_name]
    theta = np.arccos(np.clip(np.dot(cur_relative_vec,pre_relative_vec) / (np.linalg.norm(cur_relative_vec) * np.linalg.norm(pre_relative_vec)), -1, 1))  ## calculate angle
    rho = -int(np.sign(np.cross(cur_relative_vec[:2], pre_relative_vec[:2]))) ## whether target located on left or right
    KA = 50 # 转角最大为50度
    return 0 if theta < 0.002 else rho * np.clip(theta * K, 0, np.pi) / np.pi * 0.5 * KA

# 判断是否结束
def judge(obs):
    done = False
    info = False
    r_b_distance = np.linalg.norm(obs["Escaper"][:2]-obs["Catcher"][:2])     
    r_g_distance = np.linalg.norm(obs["Escaper"][:2]-obs["Goal"][:2])     
    print(f"距离终点{r_g_distance}m, 两车相距{r_b_distance}m")

    if r_b_distance < 8:
        # Catcher catch escaper
        done = True
        info = "Being caught"
    elif r_g_distance < 15:
        done = True
        info = "Reach goal"
    return info, done

if __name__ == '__main__':
    UE_ip = "127.0.0.1"

    car_name= ["Escaper","Catcher"]
    car_manul_control_flag = [True,False]
    car_origin_pos = [[0,0,0],[0,210,0]]
    car_agent = {}
    record_agent = {}
    # 初始化智能体
    for i in range(len(car_name)):
        car_agent[car_name[i]] = AirSimCarAgent(ip = UE_ip, vehicle_name= car_name[i], control_flag= car_manul_control_flag[i],origin_pos= car_origin_pos[i])
        record_agent[car_name[i]] = AirSimCarAgent(ip = UE_ip, vehicle_name= car_name[i], control_flag= car_manul_control_flag[i],origin_pos= car_origin_pos[i])
        thread = Thread(target=record_agent[car_name[i]].record_car_state, daemon=True)
        thread.start()

    # 给目录添加时间戳
    timelabel = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime(time.time()))
    rootpath = os.path.join("D:/Qiyuan/Record", timelabel)
    for agent in record_agent.values():
        agent.startRecording(rootpath)

    # 初始化车辆动作状态
    action = {}
    action["Catcher"] = {"throttle": 1,"steering": 0,"brake": 0}
    car_agent["Catcher"].do_action(action["Catcher"])

    # 当前状态
    cur_obs = {}
    pre_obs = {}
    done = False
    while not done:
        if pre_obs:
            # 计算拦截者的转角并执行
            action["Catcher"]["steering"] = Propotional_navi(cur_obs, pre_obs, "Catcher", 100)
            car_agent["Catcher"].do_action(action["Catcher"])
        pre_obs = copy.deepcopy(cur_obs)
        time.sleep(0.05)

        for name in car_name:
            state = car_agent[name].get_car_state()
            position = np.array([state["position_x"], state["position_y"], state["position_z"]])
            cur_obs[name] = position
        # 目标点位置
        cur_obs["Goal"] = car_agent["Escaper"].get_object_pose("Goal")
        info,done = judge(cur_obs)
    print(info)
    
    for agent in record_agent.values():
        agent.stopRecording()

    # 重置
    car_agent["Escaper"].reset()
