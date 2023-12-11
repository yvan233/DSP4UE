import airsim
import time
import math
from AirSimCarAgent import AirSimCarAgent
from geo_change import *

car = AirSimCarAgent(ip = "127.0.0.1", vehicle_name= "Car0")

# 获取gps信息
geo_point = car.get_car_gps()
# 获取航向角
state = car.get_car_state()
pitch, roll, yaw = state["orientation"]
yaw = math.degrees(yaw)
print(f"{geo_point[0]},{geo_point[1]},{yaw}")
    
# 轨迹规划
nav = NAV() 
origin_point = wgs84_to_gcj02(geo_point[0],geo_point[1])  # wgs84转高德地图坐标
destination_point = [116.181854,40.060284]  # 目标点经纬度 高德地图坐标
origin = f"{origin_point[0]},{origin_point[1]}"
destination = f"{destination_point[0]},{destination_point[1]}"
paths = nav.amap_driving(origin,destination)
print(paths)
# paths = [(116.16873581763917, 40.05433621084417), (116.16874379507604, 40.054692205495975), (116.16873876175426, 40.054843187515196), (116.168733738844, 40.05491217421751), (116.16874966146295, 40.055878148724936), (116.16874966146295, 40.055878148724936), (116.17123466728687, 40.05581277817065), (116.17193463411196, 40.05579307797319), (116.17347795849624, 40.055741935559176), (116.17384396378834, 40.055866603416284), (116.1739402145042, 40.05600577280733), (116.17471523316775, 40.05710213569763), (116.17533383952049, 40.05798221995173), (116.17561354017722, 40.058572697960045), (116.17571877995577, 40.05898186670357)]

i = 0
k = 1 # 比例系数
action = {"throttle": 0.8,"steering": 0,"brake": 0}

while i < len(paths):
    # 获取gps信息、速度、航向角
    current_point = car.get_car_gps()
    state = car.get_car_state()
    speed = state["speed"]
    pitch, roll, yaw = state["orientation"]
    yaw = math.degrees(yaw)

    # 前瞻点
    ahead_point = paths[i]
    # 计算距离和角度
    distance = geodistance(current_point[0],current_point[1],ahead_point[0],ahead_point[1])
    angle = geoangle(current_point[0],current_point[1],ahead_point[0],ahead_point[1])
    # 计算角度差
    angle_err = angle-yaw
    # 速度控制
    if speed > 16:
        action["throttle"] = 0.7
    elif speed < 14:
        action["throttle"] = 0.8

    if distance < 20:
        print("next point")
        i += 1
    else:
        # 转角控制
        action["steering"] = k*angle_err
        if action["steering"] > 50:
            action["steering"] = 50
        elif action["steering"] < -50:
            action["steering"] = -50
    print(f"距离{distance}m,角度{round(angle,2)}°,航向角{round(yaw,2)}°")
    car.do_action(action)
    time.sleep(0.1)

print("reach the destination")
action["brake"] = 1
car.do_action(action)