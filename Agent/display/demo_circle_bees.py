# 小蜜蜂八字飞行demo
import sys
sys.path.insert(1,".")
from Agent.AirSimUavAgent import AirSimUavAgent
import airsim
import numpy as np
import math
import time
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
origin_pos = [[-28, -45, -22],
            [-28, -44.75, -22],
            [-28, -44.5, -22],
            [-28, -44.25, -22],
            [-28.25, -45, -22],
            [-28.25, -44.75, -22],
            [-28.25, -44.5, -22],
            [-28.25, -44.25, -22]]

origin_geopoint = (116.16872381923261, 40.05405620434274,150)

uav0 = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= "Uav0", origin_pos=origin_pos[0])
uav1 = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= "Uav1", origin_pos=origin_pos[1])
uav2 = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= "Uav2", origin_pos=origin_pos[2])
uav3 = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= "Uav3", origin_pos=origin_pos[3])
uav4 = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= "Uav4", origin_pos=origin_pos[4])
uav5 = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= "Uav5", origin_pos=origin_pos[5])
uav6 = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= "Uav6", origin_pos=origin_pos[6])
uav7 = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= "Uav7", origin_pos=origin_pos[7])

radius = 1
center = [-28+3,-44.5]
direction = [1,0.268] # tan15°
altitude = 1+22

uav0.take_off()

time.sleep(2.55)
uav1.take_off()
move_eight(uav0, radius, center, altitude, direction, velocity = 0.6, waited = False, iterations = 5)

time.sleep(2.55)
uav2.take_off()
move_eight(uav1, radius, center, altitude, direction, velocity = 0.6, waited = False, iterations = 5)

time.sleep(2.55)
uav3.take_off()
move_eight(uav2, radius, center, altitude, direction, velocity = 0.6, waited = False, iterations = 5)

time.sleep(2.55)
uav4.take_off()
move_eight(uav3, radius, center, altitude, direction, velocity = 0.6, waited = False, iterations = 5)

time.sleep(2.5)
uav5.take_off()
move_eight(uav4, radius, center, altitude, direction, velocity = 0.6, waited = False, iterations = 5)

time.sleep(2.55)
uav6.take_off()
move_eight(uav5, radius, center, altitude, direction, velocity = 0.6, waited = False, iterations = 5)

time.sleep(2.55)
uav7.take_off()
move_eight(uav6, radius, center, altitude, direction, velocity = 0.6, waited = False, iterations = 5)

time.sleep(2.55)
move_eight(uav7, radius, center, altitude, direction, velocity = 0.6, waited = True, iterations = 5)

uav0.land(waited = True)