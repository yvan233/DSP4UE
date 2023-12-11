import airsim
import numpy as np
import os
import time
import cv2
import csv
import sys
import socket
from threading import Thread
from pymap3d import ned2geodetic,geodetic2ned

def gps2airsim(lon,lat,h,lon0,lat0,h0):
    '''
    说明：GPS（WGS84）坐标转Airsim坐标
    输入：
     - lat: float，纬度
     - lon: float，经度
     - h: float，高程
     - lon0: float，player start的经度
     - lat0: float，player start的纬度
     - h0: float，player start的高程
    输出：
     - ax: float，东E
     - ay: float，北N
     - az: float，天U
    '''
    ax,ay,az = geodetic2ned(lat,lon,h,lat0,lon0,h0)
    
    return airsim.Vector3r(ax,ay,az)

class AirSimUavAgent():
    """
    AirSimUavAgent用于在AirSim模拟器上实现无人机的控制，并记录控制过程中的图片，以及无人机的状态和碰撞信息。
    实现的功能包括：
        1. 初始化AirSim模拟器
        2. 获取无人机的状态信息
        3. 获取无人机的碰撞信息
        4. 控制无人机
        5. 获取图像数据
        6. 获取雷达数据
        7. 记录无人机的控制过程
        8. 停止记录
        9. 重置无人机
        
    """
    def __init__(self, origin_geopoint, ip = "", vehicle_name = "", control_flag = False, origin_pos = [0, 0, 0]):
        image_size = (84, 84, 1)  # image shape for default gym env
        ## name
        self.name = vehicle_name
        self.origin_pos = origin_pos
        self.origin_geopoint = origin_geopoint
        self.header = ['timestamp', 'position', 'orientation', 'linear_velocity', 'linear_acceleration', 'angular_velocity', 'angular_acceleration']
        self.connect(ip, control_flag)


    def connect(self, ip, control_flag):
        # 连接AirSim模拟器
        self.uav = airsim.MultirotorClient(ip=ip)
        self.uav.confirmConnection()
        self.uav.enableApiControl(True,self.name) if not control_flag else self.uav.enableApiControl(False,self.name)
        self.uav.armDisarm(True,self.name) if not control_flag else self.uav.enableApiControl(False,self.name)
        time.sleep(0.01)

    def get_state(self):
        # 获取无人机状态
        DIG = 6
        State = self.uav.getMultirotorState(self.name)
        kinematics = State.kinematics_estimated
        state = {
            "timestamp":str(State.timestamp),
            "position":[round(ele,DIG)+ self.origin_pos[i] for i,ele in enumerate(kinematics.position.to_numpy_array().tolist())],
            "orientation":[round(i,DIG) for i in airsim.to_eularian_angles(kinematics.orientation)],
            "linear_velocity":[round(i,DIG) for i in kinematics.linear_velocity.to_numpy_array().tolist()],
            "linear_acceleration":[round(i,DIG) for i in kinematics.linear_acceleration.to_numpy_array().tolist()],
            "angular_velocity":[round(i,DIG) for i in kinematics.angular_velocity.to_numpy_array().tolist()],
            "angular_acceleration":[round(i,DIG) for i in kinematics.angular_acceleration.to_numpy_array().tolist()]
        }
        return state

    def get_gps(self):
        # 获取无人机的GPS信息
        gps = self.uav.getGpsData(vehicle_name=self.name).gnss.geo_point
        return (gps.longitude, gps.latitude, gps.altitude)

    def get_collision(self):
        # 获取无人机碰撞信息
        collision_info = self.uav.simGetCollisionInfo(self.name)
        info = {
            "has_collided":collision_info.has_collided,
            "object_name": collision_info.object_name,
        }
        return info

    def reset(self):
        # 重置无人机
        self.uav.reset()

    def take_off(self, waited=False):
        # 无人机起飞函数封装
        if waited:
            self.uav.takeoffAsync(vehicle_name = self.name).join()
        else:
            self.uav.takeoffAsync(vehicle_name = self.name)

    def land(self, waited=False):
        # 无人机降落函数封装
        if waited:
            self.uav.landAsync(vehicle_name = self.name).join()
        else:
            self.uav.landAsync(vehicle_name = self.name)

    def hover(self, waited=False):
        # 无人机悬停函数封装
        if waited:
            self.uav.hoverAsync(vehicle_name = self.name).join()
        else:
            self.uav.hoverAsync(vehicle_name = self.name)

    def go_home(self, waited=False):
        # 无人机返航函数封装
        if waited:
            self.uav.goHomeAsync(vehicle_name = self.name).join()
        else:
            self.uav.goHomeAsync(vehicle_name = self.name)

    def move_to_position(self, x, y, z, velocity, waited=False,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom, yaw_mode=airsim.YawMode()):
        # 无人机位置控制函数封装
        # 如果需要uav始终朝向速度方向需要设置 drivetrain=airsim.DrivetrainType.ForwardOnly, yaw_mode=airsim.YawMode(False, 0)
        x -= self.origin_pos[0]
        y -= self.origin_pos[1]
        z -= self.origin_pos[2]
        if waited:
            self.uav.moveToPositionAsync(x, y, z, velocity, drivetrain=drivetrain, yaw_mode=yaw_mode, vehicle_name=self.name).join()
        else:
            self.uav.moveToPositionAsync(x, y, z, velocity, drivetrain=drivetrain, yaw_mode=yaw_mode, vehicle_name=self.name)
        return

    def move_to_gps_position(self, lon, lat, h, velocity, waited=False,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom, yaw_mode=airsim.YawMode()):
        # 无人机GPS位置控制函数封装
        x,y,z = gps2airsim(lon, lat, h, self.origin_geopoint[0], self.origin_geopoint[1], self.origin_geopoint[2])
        self.move_to_position(x, y, z, velocity, waited, drivetrain, yaw_mode)

    def move_on_path(self, path, velocity,waited=False, 
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom, yaw_mode=airsim.YawMode()):
        # 无人机航路点飞行函数封装
        offset = airsim.Vector3r(self.origin_pos[0], self.origin_pos[1], self.origin_pos[2])
        for i in range(len(path)):
            path[i] -= offset
        if waited:
            self.uav.moveOnPathAsync(path, velocity, drivetrain=drivetrain, yaw_mode=yaw_mode, vehicle_name=self.name).join()
        else:
            self.uav.moveOnPathAsync(path, velocity, drivetrain=drivetrain, yaw_mode=yaw_mode, vehicle_name=self.name)
        return

    def move_on_gps_path(self, path, velocity, waited=False,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom, yaw_mode=airsim.YawMode()):
        # 无人机GPS航路点飞行函数封装
        # 转换为airsim坐标
        paths = []
        for ele in path:
            paths.append(gps2airsim(ele[0], ele[1], ele[2], self.origin_geopoint[0], self.origin_geopoint[1], self.origin_geopoint[2]))
        self.move_on_path(paths, velocity, waited, drivetrain, yaw_mode)

    def move_by_velocity(self, vx, vy, vz, duration, waited=False, 
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom, yaw_mode=airsim.YawMode()):
        # 无人机速度控制函数封装
        if waited:
            self.uav.moveByVelocityAsync(vx, vy, vz, duration, drivetrain=drivetrain, yaw_mode=yaw_mode, vehicle_name=self.name).join()
        else:
            self.uav.moveByVelocityAsync(vx, vy, vz, duration, drivetrain=drivetrain, yaw_mode=yaw_mode,vehicle_name=self.name)


    def move_by_velocity_z(self, vx, vy, z, duration, waited=False, 
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom, yaw_mode=airsim.YawMode()):
        # 无人机固定高度速度控制函数封装
        if waited:
            self.uav.moveByVelocityZAsync(vx, vy, z-self.origin_pos[2], duration, drivetrain=drivetrain, yaw_mode=yaw_mode, vehicle_name=self.name).join()
        else:
            self.uav.moveByVelocityZAsync(vx, vy, z-self.origin_pos[2], duration, drivetrain=drivetrain, yaw_mode=yaw_mode,vehicle_name=self.name)


    def get_image(self, camera_name = "front_center"):
        # 获取无人机图像
        # camera_name: 无人机相机名称, 可设置成"front_center","front_right","front_left","bottom_center"和"back_center"
        img_response = self.uav.simGetImage(camera_name, airsim.ImageType.Scene, self.name) 
        np_response_image = np.asarray(bytearray(img_response), dtype="uint8")
        decoded_frame = cv2.imdecode(np_response_image, cv2.IMREAD_COLOR)
        # cv2.imwrite("result.jpg", decoded_frame)
        ret, encoded_jpeg = cv2.imencode('.jpg', decoded_frame)
        # data = encoded_jpeg.tobytes()
        # image = cv2.imdecode(encoded_jpeg, cv2.IMREAD_COLOR)
        # 解码
        # imgstring = np.asarray(bytearray(data), dtype="uint8")
        # image = cv2.imdecode(imgstring, cv2.IMREAD_COLOR)
        # return image, data
        return encoded_jpeg

if __name__ == '__main__':
    UE_ip = "127.0.0.1"
    origin_pos = [0, 0, -2]
    
    origin_geopoint = (116.16872381923261, 40.05405620434274,150)
    gps_path = [(116.16872384295665, 40.054056148557756, 152.48892211914062),
        (116.16872384295618, 40.05401209838322, 152.55076599121094),
        (116.16878122751716, 40.0540112242594, 152.6374053955078),
        (116.16878256197862, 40.05405523280805, 152.73081970214844),
        (116.16872517033799, 40.05405619907175, 152.8238983154297)]

    uav = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= "Uav0", origin_pos=origin_pos)

    print(uav.get_state())
    print(uav.get_collision())
    print(uav.get_gps())

    uav.take_off(waited = True)
    uav.move_on_gps_path(gps_path, velocity=1, waited=True)

    uav.land(waited = True)