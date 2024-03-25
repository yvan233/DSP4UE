# 特朗普位置：116.16822825174731, 40.05380267711057, 173.1387481689453
# startplayer 位置，-39340 6175 600
import sys
sys.path.insert(1,".")
from Agent.AirSimUavAgent import AirSimUavAgent
import airsim
import numpy as np
import math

UE_ip = "127.0.0.1"
origin_pos = [0, 0, -2]

origin_geopoint = (116.16872381923261, 40.05405620434274,150)

uav = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= "Uav0", origin_pos=origin_pos)
state = uav.get_state()
print(state)
# 清空画布
uav.uav.simFlushPersistentMarkers()
uav.take_off(waited = True)

uav.move_by_velocity(1, -1, -1, duration = 5, yaw_mode=airsim.YawMode(True, 0))

# uav.uav.moveByVelocityBodyFrameAsync(1, 1, -1, 5, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(True, 0), vehicle_name="Uav0")

# uav.hover(3)
# uav.move_to_position(10,10,-2,3)
while True:
    state = uav.get_state()
    points = [airsim.Vector3r(state['position'][0], state['position'][1], state['position'][2])]
    uav.uav.simPlotPoints(points, color_rgba=[0, 1, 0, 1], size=3, is_persistent=True)

