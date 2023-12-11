# 小蜜蜂楼外进入楼内脚本demo
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
		"X": -16, "Y": -76, "Z": -2,
		"Yaw": 90
	  }
	}
}
"""
import requests
import json
import sys
sys.path.insert(1,".")  # 把上一级目录加入搜索路径
from Agent.AirSimUavAgent import AirSimUavAgent
import time
UE_ip = "127.0.0.1"
origin_pos = [-16, -76, -2]

origin_geopoint = (116.16872381923261, 40.05405620434274,150)
# 进入楼内
gps_path = [(116.16794445523611,40.05388613206607,154.67825317382812),
(116.16794409043491,40.05388555746181,162.2622528076172),
(116.16794409043491,40.05388555746181,172.6),
(116.16794503279978,40.05388733664743,172.6),
(116.16799008746888,40.05388969166288,172.6),
(116.16800331501977,40.05384891056716,172.6)]

# 潜入安全屋
gps_path2 = [(116.16802368627238,40.05382577764604,172.6),
(116.16805144309774,40.053815079032574,172.6),
(116.16808614805524,40.053820688495485,172.6),
(116.16815344186337,40.053843078893244,172.6),
(116.16818779968486,40.05383736990496,172.6),
(116.16822004275155,40.05383414591224,172.6)]

# 击杀vip
gps_path3 = [(116.16822576730378,40.05383881194933,172.6),
(116.16823096746113,40.053836122787125,172.6),
(116.16823412386753,40.053825128382215,172.6),
(116.16823293216713,40.05381660889495,172.6)]

uav = AirSimUavAgent(origin_geopoint, ip = UE_ip, vehicle_name= "Uav0", origin_pos=origin_pos)

print(uav.get_state())
print(uav.get_collision())
print(uav.get_gps())

uav.take_off(waited = True)

# 进入楼内
uav.move_on_gps_path(gps_path, velocity=2.5, waited=True)
time.sleep(1.5)

# 潜入安全屋
uav.move_on_gps_path(gps_path2, velocity=1, waited=True)
# 让特朗普走
data_json = {"data":{"TP_AI":{"data":{"option":1,"control":{"forward":0.2,"right":0.0}}}}}
head = {"Content-Type": "application/json; charset=UTF-8", 'Connection': 'close'}
url ="http://127.0.0.1:5000/api/control_trump"
r = requests.post(url,json=data_json,headers=head)
if r.status_code == 200:
    data = json.loads(r.text)
    print(data)

time.sleep(1)
uav.uav.rotateByYawRateAsync(25,90/25)
time.sleep(0.5)

data_json = {"data":{"TP_AI":{"data":{"option":1,"control":{"forward":0.0,"right":0.0}}}}}
head = {"Content-Type": "application/json; charset=UTF-8", 'Connection': 'close'}
url ="http://127.0.0.1:5000/api/control_trump"
r = requests.post(url,json=data_json,headers=head)
if r.status_code == 200:
    data = json.loads(r.text)
    print(data)
time.sleep(3)

# 特朗普往回走
data_json = {"data":{"TP_AI":{"data":{"option":1,"control":{"forward":-0.2,"right":0}}}}}
head = {"Content-Type": "application/json; charset=UTF-8", 'Connection': 'close'}
url ="http://127.0.0.1:5000/api/control_trump"
r = requests.post(url,json=data_json,headers=head)
if r.status_code == 200:
    data = json.loads(r.text)
    print(data)

time.sleep(1)

uav.move_on_gps_path(gps_path3, velocity=1.5, waited=False)

time.sleep(0.5)
data_json = {"data":{"TP_AI":{"data":{"option":1,"control":{"forward":0.0,"right":0.0}}}}}
head = {"Content-Type": "application/json; charset=UTF-8", 'Connection': 'close'}
url ="http://127.0.0.1:5000/api/control_trump"
r = requests.post(url,json=data_json,headers=head)
if r.status_code == 200:
    data = json.loads(r.text)
    print(data)

time.sleep(3)
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