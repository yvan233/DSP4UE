import requests
import json
import time
from pymongo import MongoClient
import sys
from datetime import datetime

#-------------------发送命令到指定地址--------------------------------
def change_send(data_json):
    '''
    控制命令控制仿真系统
    '''
    head = {"Content-Type": "application/json; charset=UTF-8", 'Connection': 'close'}
    url = "http://127.0.0.1:8016/digital/sim/control/system/"
    data_json["time"] = int(time.time()*1000)
    r = requests.post(url,json=data_json,headers=head)
    if r.status_code == 200:
        data = json.loads(r.text)
        return data
    
def control_send(iff,data_json):
    '''
    控制命令控制智能体
    '''
    head = {"Content-Type": "application/json; charset=UTF-8", 'Connection': 'close'}
    url = "http://127.0.0.1:8016/digital/sim/control/agent/{}/".format(iff)
    data_json["time"] = int(time.time()*1000)
    r = requests.post(url,json=data_json,headers=head)
    if r.status_code == 200:
        data = json.loads(r.text)
        return data
    
def get_data_send(iff,data_json):
    '''
    获取智能体状态数据
    '''
    head = {"Content-Type": "application/json; charset=UTF-8", 'Connection': 'close'}
    url = "http://127.0.0.1:8016/digital/sim/status/data/{}/".format(iff)
    data_json["time"] = int(time.time()*1000)
    r = requests.post(url,json=data_json,headers=head)
    if r.status_code == 200:
        data = json.loads(r.text)
        return data
    
def get_info_send(iff,data_json):
    '''
    获取智能体状态数据
    '''
    head = {"Content-Type": "application/json; charset=UTF-8", 'Connection': 'close'}
    url = "http://127.0.0.1:8016/digital/sim/info/{}/".format(iff)
    data_json["time"] = int(time.time()*1000)
    r = requests.post(url,json=data_json,headers=head)
    if r.status_code == 200:
        data = json.loads(r.text)
        return data

#---------------------UE控制相关----------------------------------
def game_restart():
    return change_send({"id":"123","method":"GameRestart","params":{"uuid":"123"}})

def game_stop():
    return change_send({"id":"123","method":"GameQuit","params":{"uuid":"123"}})

def game_pause(Paused): #，1为游戏暂停，0为游戏继续
    return change_send({"id":"123","method":"GamePause","params":{"uuid":"123","Paused":Paused}})

def update():
    return change_send({"id":"123","method":"update","params":{}})

def config_ue(task_name):
    return change_send({"id":"123","method":"config_ue","params":{"task_name":task_name}})

def change_map(map_name):
    return change_send({"id":"123","method":"GameMap","params":{"uuid":"123","MapName":map_name}})

def change_wave(wave):
    change_send({"id":"123","method":"ChangeWave","params":{"uuid":"123","Option":int(wave)}})

def change_weather(weather):
    return change_send({"id":"123","method":"ChangeWeather","params":{"uuid":"123","Option":weather}})



#----------------------智能体生成相关----------------------------------
def create_tank(AgentName,AgentIff,Leader,LeaderName,Position,Attitude,
                LifeValue,ShellCapacity,SmokeShellCapacity,FiringAbility,
                FillingTime,VisualSensorAbility):
    return change_send({"id":"123","method":"CreateTank",
                        "params":{
                        "AgentName":AgentName,
                        "AgentIff":AgentIff,
                        "Leader":Leader,
                        "LeaderName":LeaderName,
                        "Position":Position,
                        "Attitude":Attitude,
                        "LifeValue": LifeValue,
                        "ShellCapacity": ShellCapacity,
                        "SmokeShellCapacity": SmokeShellCapacity,
                        "FiringAbility": FiringAbility,
                        "FillingTime": FillingTime,
                        "VisualSensorAbility": VisualSensorAbility}})

def create_drone(AgentName, AgentIff, Leader, LeaderName, Position, Attitude, LifeValue, VisualSensorAbility):
    return change_send({"id":"123","method":'CreateQuadrotor',
                        "params":{
                        "AgentName":AgentName,
                        "AgentIff":AgentIff,
                        "Leader":Leader,
                        "LeaderName":LeaderName,
                        "Position":Position,
                        "Attitude":Attitude,
                        "LifeValue": LifeValue,
                        "VisualSensorAbility": VisualSensorAbility}
                        })

def create_advanced_missile(AgentName,AgentIff,Leader,LeaderName,Position,Attitude,
                                   AttackProbability,BaseDamage,DamageRadius):
    return change_send({"id":"123","method":"CreateAdvancedMissile",
                        "params":{
                        "AgentName":AgentName,
                        "AgentIff":AgentIff,
                        "Leader":Leader,
                        "LeaderName":LeaderName,
                        "Position":Position,
                        "Attitude":Attitude,
                        "AttackProbability": AttackProbability,
                        "BaseDamage": BaseDamage,
                        "DamageRadius": DamageRadius}})

def create_missile(AgentName,AgentIff,Leader,LeaderName,Position,Attitude,
                                   AttackProbability,BaseDamage,DamageRadius):
    return change_send({"id":"123","method":"CreateMissile",
                        "params":{
                        "AgentName":AgentName,
                        "AgentIff":AgentIff,
                        "Leader":Leader,
                        "LeaderName":LeaderName,
                        "Position":Position,
                        "Attitude":Attitude,
                        "AttackProbability": AttackProbability,
                        "BaseDamage": BaseDamage,
                        "DamageRadius": DamageRadius}})

def create_guided_missile(AgentName,AgentIff,Leader,LeaderName,Position,Attitude,
                                   AttackProbability,BaseDamage,DamageRadius,MaxSpeed,AttackDistance):
    return change_send({"id":"123","method":"CreateGuidedMissile",
                        "params":{
                        "AgentName":AgentName,
                        "AgentIff":AgentIff,
                        "Leader":Leader,
                        "LeaderName":LeaderName,
                        "Position":Position,
                        "Attitude":Attitude,
                        "AttackProbability": AttackProbability,
                        "BaseDamage": BaseDamage,
                        "DamageRadius": DamageRadius,
                        "MaxSpeed":MaxSpeed,
                        "AttackDistance":AttackDistance}})

def create_ship_054A(AgentName,AgentIff,Leader,LeaderName,Position,Attitude,IsBias,LifeValue):
    return change_send({"id":"123","method":"CreateShip054A",
                        "params":{
                        "AgentName":AgentName,
                        "AgentIff":AgentIff,
                        "Leader":Leader,
                        "LeaderName":LeaderName,
                        "Position":Position,
                        "Attitude":Attitude,
                        "IsBias": IsBias,
                        "LifeValue": LifeValue}})

def create_bunker(AgentName,AgentIff,Leader,LeaderName,Position,Attitude,
                Scale,Life =1.0):
    return change_send({"id":"123","method":"CreateFixedBunker",
                        "params":{
                        "AgentName":AgentName,
                        "AgentIff":AgentIff,
                        "Leader":Leader,
                        "LeaderName":LeaderName,
                        "Position":Position,
                        "Attitude":Attitude,
                        "Scale":Scale,
                        "Life": Life}})

def create_uuv(AgentName,AgentIff,Leader,LeaderName,Position,Attitude,IsBias,LifeValue):
    return change_send({"id":"123","method":"CreateUUV",
                        "params":{
                        "AgentName":AgentName,
                        "AgentIff":AgentIff,
                        "Leader":Leader,
                        "LeaderName":LeaderName,
                        "Position":Position,
                        "Attitude":Attitude,
                        "IsBias": IsBias,
                        "LifeValue": LifeValue}})

def create_amphibian_tank(AgentName,AgentIff,Leader,LeaderName,Position,Attitude,
                LifeValue,ShellCapacity,SmokeShellCapacity,FiringAbility,
                FillingTime,VisualSensorAbility):
    return change_send({"id":"123","method":"CreateAmphibianTank",
                        "params":{
                        "AgentName":AgentName,
                        "AgentIff":AgentIff,
                        "Leader":Leader,
                        "LeaderName":LeaderName,
                        "Position":Position,
                        "Attitude":Attitude,
                        "LifeValue": LifeValue,
                        "ShellCapacity": ShellCapacity,
                        "SmokeShellCapacity": SmokeShellCapacity,
                        "FiringAbility": FiringAbility,
                        "FillingTime": FillingTime,
                        "VisualSensorAbility": VisualSensorAbility}})

def create_camera(AgentName,AgentIff,LeaderName,Position,Attitude,EquipSlot):
    return change_send({"id":"123","method":"CreateCamera",
                        "params":{
                        "AgentName":AgentName,
                        "AgentIff":AgentIff,
                        "LeaderName":LeaderName,
                        "Position":Position,
                        "Attitude":Attitude,
                        "EquipSlot":EquipSlot}})

def create_infrared_camera(AgentName,AgentIff,LeaderName,Position,Attitude,EquipSlot):
    return change_send({"id":"123","method":"CreateInfraredCamera",
                        "params":{
                        "AgentName":AgentName,
                        "AgentIff":AgentIff,
                        "LeaderName":LeaderName,
                        "Position":Position,
                        "Attitude":Attitude,
                        "EquipSlot":EquipSlot}})

def create_communication(AgentName,AgentIff,LeaderName,Position,Attitude):
    return change_send({"id":"123","method":"CreateCommunication",
                        "params":{
                        "AgentName":AgentName,
                        "AgentIff":AgentIff,
                        "LeaderName":LeaderName,
                        "Position":Position,
                        "Attitude":Attitude}})

def create_lidar(AgentName,AgentIff,LeaderName,Position,Attitude,Channels,Range,
                Points,Frequency,HFovStart,HFovEnd,VFovUpper,VFovLower,Draw,EquipSlot):
    return change_send({"id":"123","method":"CreateLidar",
                        "params":{
                        "AgentName":AgentName,
                        "AgentIff":AgentIff,
                        "LeaderName":LeaderName,
                        "Position":Position,
                        "Attitude":Attitude,
                        "Channels":Channels,
                        "Range":Range,
                        "Points":Points,
                        "Frequency":Frequency,
                        "HFovStart":HFovStart,
                        "HFovEnd":HFovEnd,
                        "VFovUpper":VFovUpper,
                        "VFovLower":VFovLower,
                        "Draw":Draw,
                        "EquipSlot":EquipSlot}})

def create_actor(actor_name,actor_class,position,attitude,scale=1):
    Lon, Lat, Alt = position
    Roll, Yaw, Pitch = attitude
    return change_send({"id":"123", "method":"CreateActor", "params":{"uuid":"123",
                        "ActorName":actor_name, "ActorClass":actor_class,
                        "Lon":float(Lon), "Lat":float(Lat), "Alt":float(Alt), 
                        "Roll":Roll, "Yaw":Yaw, "Pitch":Pitch,"Scale":scale}})



#-----------------------智能体控制相关----------------------------------
def go_forward(LeaderName,AgentName,Speed):
    return control_send("red",{"id":"123", "method":"ForwardControl", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                          "AgentName":AgentName,"Speed":Speed}})

def turn_left(LeaderName,AgentName):
    return control_send("red",{"id":"123", "method":"RotationControl", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                           "AgentName":AgentName,"Angle":-90.0,"Speed":20.0}})

def turn_right(LeaderName,AgentName):
    return control_send("red",{"id":"123", "method":"RotationControl", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                           "AgentName":AgentName,"Angle":90.0,"Speed":20.0}})
def fire(LeaderName,AgentName):
    return control_send("red",{"id":"123", "method":"FireShell", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                           "AgentName":AgentName}})

def smoke(LeaderName,AgentName):
    return control_send("red",{"id":"123", "method":"FireSmokeShell", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                           "AgentName":AgentName}})

def open_sight(open,LeaderName,AgentName):
    return control_send("red",{"id":"123", "method":"OpenSight", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                     "AgentName":AgentName,"Open":open}}) 

def launch_missile(LeaderName,AgentName, Position):
    return control_send("red",{"id":"123", "method":"LaunchMissile", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                           "AgentName":AgentName,"Position":Position}})

def mis_angular_velocity( LeaderName, AgentName, AngularVelocity):
    return control_send("red",{"id":"123", "method":"AngularVelocityControl", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                           "AgentName":AgentName,"AngularVelocity":AngularVelocity}})

def mis_position_control(LeaderName,AgentName, Position,Attitude):
    return control_send("red",{"id":"123", "method":"PositionControlMissile", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                           "AgentName":AgentName,"Position":Position,"Attitude":Attitude}})

def change_cam(open,LeaderName,AgentName):
    return control_send("red",{"id":"123", "method":"ChangeCamera", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                        "AgentName":AgentName,"Open":open}})
def open_cam(open,LeaderName,AgentName):
    return control_send("red",{"id":"123", "method":"OpenCamera", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                        "AgentName":AgentName,"Open":open}})
def export_cam_fig(open,LeaderName,AgentName):
    return control_send("red",{"id":"123", "method":"ExportCameraFigure", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                              "AgentName": AgentName,"Export":open,"Generate":open}})    

def open_icam(open,LeaderName,AgentName):
    return control_send("red",{"id":"123", "method":"OpenCamera", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                        "AgentName":AgentName,"Open":open}})
def export_icam_fig(open,LeaderName,AgentName):
    return control_send("red",{"id":"123", "method":"ExportCameraFigure", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                              "AgentName":AgentName,"Export":open,"Generate":open}})

def open_sonar(open,LeaderName,AgentName):
    return control_send("red",{"id":"123", "method":"OpenSonar", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                        "AgentName":AgentName,"Open":open}})

def open_lidar(open,LeaderName,AgentName):
    return control_send("red",{"id":"123", "method":"OpenLidar", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                              "AgentName": AgentName,"Open":open}})

def open_imu(open,LeaderName,AgentName):
    return control_send("red",{"id":"123", "method":"OpenImu", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                              "AgentName": AgentName,"Open":open}})
def set_fov(LeaderName,AgentName,Fov):
    return control_send("red",{"id":"123", "method":"CameraFov", "params":{"uuid":"123","LeaderName":LeaderName,
                                                                        "AgentName":AgentName,"Fov":Fov}})
#-----------------------------------------------0
def change_model_drone(LeaderName,AgentName,Open):
    return control_send("red", {"id":"123", "method":"ChangeModelQD", 
                         "params":{"uuid":"123","LeaderName":LeaderName, "AgentName":AgentName,"Open":Open}}
    )

def velocity_control_drone(LeaderName,AgentName,Vx, Vy, Vz):
    return control_send(
        "red", {"id":"123", "method":"VelocityControlQD", 
                         "params":{"uuid":"123","LeaderName":LeaderName, "AgentName":AgentName,
                                   "Vx":Vx, 'Vy':Vy, 'Vz':Vz}}
    )

def change_camera_drone(LeaderName,AgentName,Open):
    return control_send(
        "red", {"id":"123", "method":"ChangeCameraQD", 
                         "params":{"uuid":"123","LeaderName":LeaderName, "AgentName":AgentName,
                                   "Open":Open}}
    )

def front_camera_control_drone(LeaderName,AgentName,AngleYaw,AnglePitch,Speed):
    return control_send(
        "red", {"id":"123", "method":"FrontCameraControlQD", 
                         "params":{"uuid":"123","LeaderName":LeaderName, "AgentName":AgentName,
                                   "AngleYaw":AngleYaw, "AnglePitch":AnglePitch, "Speed":Speed}}
    )

def rotation_control_drone(LeaderName,AgentName,AngleYaw,Speed):
    return control_send(
        "red", {"id":"123", "method":"RotationControlQD", 
                         "params":{"uuid":"123","LeaderName":LeaderName, "AgentName":AgentName,
                                   "AngleYaw":AngleYaw, "Speed":Speed}}
    )


def detination_move_drone(LeaderName,AgentName, Lon, Lat, Alt, Speed):
    return control_send(
        "red", {"id":"123", "method":"DestinationMoveQD", 
                         "params":{"uuid":"123","LeaderName":LeaderName, "AgentName":AgentName,
                                   "Lon":Lon, "Lat":Lat, "Alt":Alt, "Speed":Speed}}
    )
#-----------------------------------------------1

def open_multi_view(open):
    return change_send({"id":"123","method":"OpenMultiView","params":{"uuid":"123","Open":open}})

def agent_multi_view(open,agent_name):
    return change_send({"id":"123","method":"AgentMultiView","params":{"uuid":"123","Open":open,"AgentName":agent_name}})["data"]



#--------------------------------数据获取相关-----------------------------------
def get_agent_data(LeaderName, AgentName):
    return get_data_send("red",{"id": "123", "method": "get_agent_data", "params": {"LeaderName": LeaderName, "AgentName": AgentName}})["data"]
    
def get_agent_state(LeaderName,AgentName):
    return get_info_send("red",{"id":"123", "method":"get_agent_state", "params":{"LeaderName":LeaderName,"AgentName":AgentName}})["data"]

def get_agent_operation(LeaderName,AgentName):
    return get_info_send("red",{"id":"123", "method":"get_agent_operation", "params":{"LeaderName":LeaderName,"AgentName":AgentName}})["data"]

def get_agent_leader(AgentName):
    return get_info_send("blue",{"id":"123", "method":"get_agent_leader", "params":{"agent_name":AgentName}}) ["data"]
 
def get_agent_class(AgentName):
    return get_info_send("blue",{"id":"123", "method":"get_agent_class", "params":{"agent_name":AgentName}})["data"]
  
def get_agent_iff(AgentName):
    return get_info_send("blue",{"id":"123", "method":"get_agent_iff", "params":{"agent_name":AgentName}})["data"]  

def get_agent_position(AgentName):
    return get_data_send("red",{"id":"123", "method":"get_agent_base_data", "params":{"LeaderName":get_agent_leader(AgentName),"AgentName":AgentName}})["data"][0][:2]

def get_agents_avaliable():
    return change_send({"id":"123", "method":"get_state_data_avaliable_agents_all", "params":{}})["data"]