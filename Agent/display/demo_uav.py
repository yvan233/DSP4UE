# 无人机侦察demo
"""
{
	"SettingsVersion": 1.2,
	"ViewMode":"SpringArmChase",
	"ClockType": "ScalableClock",
	"ClockSpeed": 1,
	"OriginGeopoint": {
	  "Latitude": 40.05405620434274,
	  "Longitude": 116.16872381923261, 
	  "Altitude": 150
	},
	"Vehicles": {
	  "Uav0": {
		"VehicleType": "SimpleFlight",
		"X": 595, "Y": 0, "Z": -2,
		"Yaw": 180
	  }
	}
}
"""

import sys
sys.path.insert(1,".")
from Agent.AirSimUavAgent import AirSimUavAgent
import time
UE_ip = "127.0.0.1"
origin_pos = [595, 0, -2]

origin_geopoint = (116.16872381923261, 40.05405620434274,150)
gps_path = [(116.168715,40.058383,185),
(116.168715,40.056050,185),
(116.168715,40.054030,185)]

C3_path = [(116.168715,40.054030,174),
(116.168715,40.053562,174),
(116.167900,40.053562,174),
(116.167900,40.054030,174),
(116.168715,40.054030,174)]
uav = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= "Uav0", origin_pos=origin_pos)

uav.take_off(waited = True)
uav.move_to_position(595,0,-35,velocity=5,waited=True)

uav.move_on_gps_path(gps_path, velocity=8, waited=True)

time.sleep(3)
print("Start")
uav.move_to_gps_position(C3_path[0][0],C3_path[0][1],C3_path[0][2],velocity=2,waited=True)

uav.uav.rotateByYawRateAsync(25,90/25).join()
uav.move_to_gps_position(C3_path[1][0],C3_path[1][1],C3_path[1][2],velocity=2,waited=True)

uav.uav.rotateByYawRateAsync(25,90/25).join()
uav.move_to_gps_position(C3_path[2][0],C3_path[2][1],C3_path[2][2],velocity=2,waited=True)

uav.uav.rotateByYawRateAsync(25,90/25).join()
uav.move_to_gps_position(C3_path[3][0],C3_path[3][1],C3_path[3][2],velocity=2,waited=True)

uav.uav.rotateByYawRateAsync(25,90/25).join()
uav.move_to_gps_position(C3_path[4][0],C3_path[4][1],C3_path[4][2],velocity=2,waited=True)


time.sleep(5)
uav.land(waited = True)