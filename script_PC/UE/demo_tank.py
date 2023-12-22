# Launch the system and run the distributed algorithm
import time 
import sys
sys.path.insert(1,".")
from DASP.module import Node,Moniter
from DASP.control import ControlMixin

import os
from Connection.API_Utils import *

task_name = "qiyuan_{}".format(datetime.now().strftime("%Y%m%d%H%M%S")) # 测试任务名称
print(task_name)

# 1. 配置 UE 基础信息-------0
config_ue(task_name)
#--------------------------1

nodeNum = 4  # Number of nodes
for i in range(1,nodeNum+1):
    create_tank(f'Tank_Red_{i}','red',1,f'Tank_Red_{i}',[114.496200,36.610173 - i*0.0002,8.0],[0,0,0],4000.0,5,3,1,10.0,0)
    create_camera(f"Tank_Red_Cam_{i}",'red',f'Tank_Red_{i}',[114.496200,36.610173 - i*0.0002,8.0],[0,0,0],"05")
    create_communication(f"Tank_Red_Communication_{i}",'red',f'Tank_Red_{i}',[114.496200,36.610173 - i*0.0002,8.0],[0,0,0])

time.sleep(1)
change_map("FZ_test")
time.sleep(2)
game_restart()
time.sleep(5)

update()
time.sleep(1)
change_cam(0,"Tank_Red_1","Tank_Red_1")

startNode = "1" # Starting node ID
nodelist = [] # List of node processes
controlMixin = ControlMixin("Pc") # Collection of control functions

# Launch the monitoring script
moniter = Moniter()
moniter.run()

# Launch node processes
for i in range(nodeNum):
    node = Node(i)
    nodelist.append(node)

time.sleep(2)
DappName = "Tank"
print("start task: "+DappName)
controlMixin.startTask(DappName,startNode)

moniter.wait()