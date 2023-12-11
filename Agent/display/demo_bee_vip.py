# 小蜜蜂击杀vip demo
# vip位置：116.16822825174731, 40.05380267711057, 173.1387481689453
# startplayer 位置，-39340 6175 600
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
	"PawnPaths": {
		"Bee":{"PawnBP":"Class'/Game/newFly/BP_FlyingPawn.BP_FlyingPawn_C'"}
		},
	"Vehicles": {
	  "Uav0": {
		"VehicleType": "SimpleFlight",
		"PawnPath": "Bee",
		"X": -28, "Y": -44, "Z": -22,
		"Yaw": 0
	  },
	  "Uav1": {
		"VehicleType": "SimpleFlight",
		"X": 5, "Y": 0, "Z": -2,
		"Yaw": 180
	  }
	}
}
"""
import time
import requests
import json
import sys
sys.path.insert(1,".")  # 把上一级目录加入搜索路径
from Agent.AirSimUavAgent import AirSimUavAgent
UE_ip = "127.0.0.1"
origin_pos = [-28, -44, -22]

origin_geopoint = (116.16872381923261, 40.05405620434274,150)
gps_path = [(116.16820743849861,40.05380467493661,173.2445),
(116.16821132795593,40.053814028496106,173.2445),
(116.16822004024276,40.05382169150999,173.2445),
(116.16822413498689,40.05383307668123,173.2445),
(116.16822872028492,40.05383557325449,173.2445)]

gps_path2 = [(116.16823340147532,40.05383504837999,173.2445),
(116.16823419719795,40.05382717734135,173.2445),
(116.168233048514,40.053821974088486,173.2445),
(116.16823339979337,40.05381496127576,173.2445)]

time.sleep(3)
print("ready")
time.sleep(1)
uav = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= "Uav0", origin_pos=origin_pos)

print(uav.get_state())
print(uav.get_collision())
print(uav.get_gps())

uav.take_off(waited = True)

uav.move_on_gps_path(gps_path, velocity=0.2, waited=True)

# 让特朗普走到椅子的位置
# data_json = {"data":{"MIS_AI":{"data":{"option":0,"destination":{"xc":-3000.0,"yc":3000.0,"zc":500.0}}},'MIS_AI2':{"data":{"option":0,"destination":{"xc":-3000.0,"yc":3000.0,"zc":500.0}}}}}
data_json = {"data":{"TP_AI":{"data":{"option":1,"control":{"forward":-0.5,"right":0.0}}}}}
head = {"Content-Type": "application/json; charset=UTF-8", 'Connection': 'close'}
url ="http://127.0.0.1:5000/api/control_trump"
r = requests.post(url,json=data_json,headers=head)
if r.status_code == 200:
    data = json.loads(r.text)
    print(data)

time.sleep(2)
# 控制特朗普坐下
data_json = {"data":{"TP_AI":{"data":{"option":2}}}}
head = {"Content-Type": "application/json; charset=UTF-8", 'Connection': 'close'}
url ="http://127.0.0.1:5000/api/control_trump"
r = requests.post(url,json=data_json,headers=head)
if r.status_code == 200:
    data = json.loads(r.text)
    print(data)

time.sleep(4)
# 特朗普起身走两步
data_json = {"data":{"TP_AI":{"data":{"option":1,"control":{"forward":0.2,"right":0.0}}}}}
head = {"Content-Type": "application/json; charset=UTF-8", 'Connection': 'close'}
url ="http://127.0.0.1:5000/api/control_trump"
r = requests.post(url,json=data_json,headers=head)
if r.status_code == 200:
    data = json.loads(r.text)
    print(data)
time.sleep(1.5)

data_json = {"data":{"TP_AI":{"data":{"option":1,"control":{"forward":0.0,"right":0.0}}}}}
head = {"Content-Type": "application/json; charset=UTF-8", 'Connection': 'close'}
url ="http://127.0.0.1:5000/api/control_trump"
r = requests.post(url,json=data_json,headers=head)
if r.status_code == 200:
    data = json.loads(r.text)
    print(data)
time.sleep(2)

uav.move_on_gps_path(gps_path2, velocity=0.2, waited=True)
# 控制特朗普倒地死亡
data_json = {"data":{"TP_AI":{"data":{"option":3}}}}
head = {"Content-Type": "application/json; charset=UTF-8", 'Connection': 'close'}
url ="http://127.0.0.1:5000/api/control_trump"
r = requests.post(url,json=data_json,headers=head)
if r.status_code == 200:
    data = json.loads(r.text)
    print(data)
time.sleep(5)
uav.land(waited = True)