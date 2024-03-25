from math import radians, cos, sin, asin, sqrt
def geodistance(location1,location2):
    """ SL
    说明：计算两个经纬高度坐标之间的距离，返回单位是 m
    输入：
        - :location1: list, [经,纬,高]， 子元素为float
        - :location2: list, [经,纬,高]， 子元素为float
    返回：两点距离，单位 米
    """
    lng1,lat1,alt1 = location1
    lng2,lat2,alt2 = location2
    lng1, lat1, lng2, lat2 = map(radians, [float(lng1), float(lat1), float(lng2), float(lat2)]) # 经纬度转换成弧度
    dlon = lng2-lng1
    dlat = lat2-lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2 
    distance = 2*asin(sqrt(a))*6371393 # 地球平均半径，6371393 m
    distance = sqrt(distance**2+(float(alt2)-float(alt1))**2)
    distance = round(distance,3)
    return distance

print(geodistance([114.496200,36.610173,8],[114.496200-0.0002,36.610173 - 0.0002,8]))

# base connection
# def get_agent_base_data(self, LeaderName, AgentName):
#     ''' 该函数由石龙编写并负责维护
#     说明：获取智能体状态数据库表基本数据，包括智能体位置、姿态、速度
#     输入：
#         - :AgentName: 智能体的名称，string
#     输出：
#         - :position: 位置，list 型
#         - :lon: 经度，float 型，float
#         - :lat: 纬度，float 型，float
#         - :alt: 高度，float 型，float
#         - :attitude: 姿态，list 型
#         - :roll: roll 角，float 型，单位°
#         - :pitch: pitch 角，float 型，单位°
#         - :yaw: yaw 角，float 型，单位°
#         - :velocity: 速度，list 型
#         - :x: 向北速度，单位 m/s
#         - :y: 向东速度，单位 m/s
#         - :z: 向上速度，单位 m/s
#     '''
#     if LeaderName != self.get_agent_leader(AgentName):
#         return False
    
#     col_name = "state_{}_{}".format(self.get_agent_iff(AgentName),AgentName)
#     col = self.db.get_collection(col_name)
#     data = col.find().sort("_id",-1)[0]

#     position = data["agent_location"]
#     position = [float(position["lon"]),float(position["lat"]),float(position["alt"])]
#     attitude = data["agent_attitude"]
#     attitude = [float(attitude["roll"]),float(attitude["pitch"]),float(attitude["yaw"])]
#     velocity = data["agent_velocity"]
#     velocity = [float(velocity["x"]),float(velocity["y"]),float(velocity["z"])]
    
#     return position, attitude,velocity