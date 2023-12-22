import requests
import json
import time
from pymongo import MongoClient
import sys
import os
from datetime import datetime
sys.path.append(".")
from Connection.API_Utils import *


if __name__ == '__main__':
    # 0. 初始化参数-------------0
    task_name = "qiyuan_{}".format(datetime.now().strftime("%Y%m%d%H%M%S")) # 测试任务名称
    print(task_name)
    #--------------------------1
    
    # 1. 配置 UE 基础信息-------0
    config_ue(task_name)
    #--------------------------1

    tank_num = 4
    for i in range(tank_num):
        
        create_tank('Tank_Red_{}'.format(i),'red',1,'Tank_Red_{}'.format(i),[114.496200,36.610173 - i*0.0002,8.0],[0,0,0],4000.0,5,3,1,10.0,0)
        create_camera("Tank_Red_Cam_{}".format(i),'red','Tank_Red_{}'.format(i),[114.496200,36.610173 - i*0.0002,8.0],[0,0,0],"05")
        create_communication("Tank_Red_Communication_{}".format(i),'red','Tank_Red_{}'.format(i),[114.496200,36.610173 - i*0.0002,8.0],[0,0,0])

    time.sleep(1)
    change_map("FZ_test")
    time.sleep(2)
    game_restart()
    time.sleep(5)
    
    update()
    time.sleep(1)

    agentname = "Tank_Red_0"
    leadername = "Tank_Red_0"
    change_cam(0,"Tank_Red_0","Tank_Red_0")
   
    # 打开IMU
    # open_imu(1,"Tank_Red_0","Tank_Red_0")
    # open_imu(1,"Tank_Red_1","Tank_Red_1")
    # time.sleep(3) #IMU初始化时间
    # 打开雷达
    # open_lidar(1,"Tank_Red_0","Tank_Red_lidar_0")
    # time.sleep(3)

    # open_cam(1,"Tank_Red_0","Tank_Red_Cam_0")
    # export_cam_fig(1,"Tank_Red_0","Tank_Red_Cam_0")

    time.sleep(4)
    go_forward('Tank_Red_0', 'Tank_Red_0',4)
    # 

    go_forward('Tank_Red_1', 'Tank_Red_1',5)
    # time.sleep(2)
    # turn_left('Tank_Red_1', 'Tank_Red_0')

    # get_agent_state("Tank_Red_0","Tank_Red_0")
    # get_agent_state('Tank_Red_1')