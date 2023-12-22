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

    num = 4
    for i in range(num):
        AgentName = f"blue_quadrotor_{i}"
        AgentIff = 'blue'
        Leader= 1
        LeaderName = f"blue_quadrotor_{i}"
        Position = (114.490456,36.607655- i*0.0002,8)
        Attitude = (0, 0, 0)
        LifeValue = 3000.0
        VisualSensorAbility = 0
        create_drone(AgentName, AgentIff, Leader, LeaderName,Position, Attitude,LifeValue,VisualSensorAbility)
        create_communication(f"blue_quadrotor_Communication_{i}",AgentIff,AgentName,Position,Attitude)

    time.sleep(1)
    change_map("FZ_test")
    time.sleep(2)
    game_restart()
    time.sleep(5)
    
    update()
    time.sleep(1)

    AgentName = "blue_quadrotor_1"
    LeaderName = "blue_quadrotor_1"
   
    #四旋翼无人机，切换镜头： 1前，2下，3右，4左，5第三人称
    Open = 1
    change_camera_drone(LeaderName,AgentName,Open)
    print("切换镜头： 1")
    time.sleep(3)

    Open = 5
    change_camera_drone(LeaderName,AgentName,Open)
    print("切换镜头： 5")
    time.sleep(3)

    # 四旋翼无人机，切换控制模块式，模式0：速度模式，模式1：位移模式
    Open = 0
    change_model_drone(LeaderName, AgentName, Open)

    #四旋翼无人机，速度模式下的移动控制
    #上移，切镜，前移，右移。
    Vx = 0
    Vy = 0
    Vz = 30
    velocity_control_drone(LeaderName, AgentName, Vx, Vy, Vz)
    time.sleep(6)
    Vx = 0
    Vy = 0
    Vz = 10
    velocity_control_drone(LeaderName, AgentName, Vx, Vy, Vz)
    time.sleep(6)

    Vx = 30
    Vy = 0
    Vz = 0
    velocity_control_drone(LeaderName, AgentName, Vx, Vy, Vz)
    time.sleep(10)
    Vx = 0
    Vy = 30
    Vz = 0
    velocity_control_drone(LeaderName, AgentName, Vx, Vy, Vz)
    time.sleep(10)
    Vx = 0
    Vy = 0
    Vz = 0
    velocity_control_drone(LeaderName, AgentName, Vx, Vy, Vz)

    #位移模式
    Open = 1
    change_model_drone(LeaderName, AgentName, Open)

    #四旋翼无人机，位移模式下的定点移动
    Lon = 114.490456
    Lat = 36.607655
    Alt = 0
    Speed = 10
    detination_move_drone(LeaderName, AgentName, Lon, Lat, Alt, Speed)
    #-------------------------1