# 小蜜蜂八字飞行demo
import sys
sys.path.insert(1,".")
from Agent.AirSimUavAgent import AirSimUavAgent
import airsim
import numpy as np
import math

def move_eight(uav, radius = 2, center = [0,0], altitude = 3, direction = [1,0], velocity = 2, waited = True, iterations = 1):
    # 以8字飞行
    # center 为8字中心
    # direction 为c1c2的方向向量
    height = -altitude
    center = np.array(center)
    direction = np.array(direction)
    direction = direction/np.linalg.norm(direction)
    direction_angle = math.atan2(direction[1], direction[0])
    circle1_center = center - direction*radius
    circle2_center = center + direction*radius
    path = []
    for i in range(100):
        theta = math.pi - math.pi / 100 * i + direction_angle
        rotation = np.array([math.cos(theta), math.sin(theta)])
        point = circle1_center + rotation * radius
        path.append(airsim.Vector3r(point[0], point[1], height))
    for i in range(200):
        theta = math.pi / 100 * i + direction_angle
        rotation = np.array([math.cos(theta), math.sin(theta)])
        point = circle2_center - rotation * radius
        path.append(airsim.Vector3r(point[0], point[1], height))
    for i in range(100):
        theta = 2*math.pi - math.pi / 100 * i + direction_angle
        rotation = np.array([math.cos(theta), math.sin(theta)])
        point = circle1_center + rotation * radius
        path.append(airsim.Vector3r(point[0], point[1], height))
    path = path * iterations
    uav.move_on_path(path, velocity=velocity, waited=waited)

UE_ip = "127.0.0.1"
origin_pos = [-28, -44, -22]

origin_geopoint = (116.16872381923261, 40.05405620434274,150)

uav = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= "Uav0", origin_pos=origin_pos)

uav.take_off(waited = True)

radius = 1
center = [2*radius-28,-44-0.5]
direction = [1,0]
altitude = 1+22
move_eight(uav, radius, center, altitude, direction, velocity = 0.6, waited = True, iterations = 5)

uav.land(waited = True)