import datetime
import numpy as np
import os
import codecs
import json
import threading
import time
import socket
from math import radians, cos, sin, asin, sqrt


class DaspMap:
    def __init__(self, port = 50001):
        self.port = port
        self.load()
        self.DistanceMatrix = self.getDistanceMatrix()

    def run(self):
        t = threading.Thread(target=self.mapServer,)    
        t.setDaemon(True)
        t.start()

    def mapServer(self):
        """
        更新节点位置，维护节点之间的拓扑关系
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        localIP = socket.gethostbyname(socket.gethostname())
        s.bind((localIP, self.port))
        print ("MapServer: {}:{}".format(localIP,self.port))
        while 1:
            try:
                data, addr = s.recvfrom(100000000)
                jdata = json.loads(data)
            except Exception:
                print ("MapServer error!")
            else:
                if jdata["key"] == "GetTopology":
                    nodeid = jdata["id"]
                    location = jdata["location"]
                    self.updateDistanceMatrix(nodeid,location)
                    nodeTopology,nodeNbrDistance = self.getNodeTopology(nodeid)
                    data = {
                        "key": "Topology",
                        "id": nodeid,
                        "time": datetime.datetime.now().strftime("%m-%d %H:%M:%S"),
                        "topology": nodeTopology,
                        "nbrDistance": nodeNbrDistance
                    }
                    s.sendto(json.dumps(data).encode('utf-8'), addr)
                    # print(nodeTopology,nodeNbrDistance)
                else:
                    print ("Data error!")

    def wait(self):
        while True:
            time.sleep(1)

    def load(self):
        """
        加载节点信息
        """
        # 加载topology文件
        nodes = []
        wiredlessNbrID = []
        location = {}
        path = os.getcwd() + "/Dapp/Base/topology.json"
        text = codecs.open(path, 'r', 'utf-8').read()
        js = json.loads(text)
        self.COMMRANGE = js["CommRange"]
        for ele in js["Nodes"]:
            if "ID" in ele:
                nodes.append(ele["ID"])
                if ele["Wireless"]:
                    location[ele["ID"]] = ele["Location"]
        self.nodes = nodes
        self.nodeLocation = location
        # print(self.nodeLocation)

    def getDistanceMatrix(self):
        """
        获取节点之间的距离矩阵
        """
        nodeNum = len(self.nodes)
        DistanceMatrix = np.zeros((nodeNum,nodeNum))
        for i in range(nodeNum):
            for j in range(nodeNum):
                if i == j:
                    DistanceMatrix[i][j] = 0
                else:
                    DistanceMatrix[i][j] = self.coordDistance(self.nodeLocation[self.nodes[i]],self.nodeLocation[self.nodes[j]])
        return DistanceMatrix

    def updateDistanceMatrix(self, NodeId, newLocation):
        """
        更新节点之间的距离矩阵
        """
        self.nodeLocation[NodeId] = newLocation
        nodeNum = len(self.nodes)
        nodeIndex = self.nodes.index(NodeId)
        for i in range(nodeNum):
            for j in range(nodeNum):
                if (i == nodeIndex or j == nodeIndex) and i != j:
                    self.DistanceMatrix[i][j] = self.coordDistance(self.nodeLocation[self.nodes[i]],self.nodeLocation[self.nodes[j]])
        return 0


    def getTopology(self):
        """
        根据节点间的距离判断连接关系，如果小于COMMRANGE则连接
        """
        nodeNum = len(self.nodes)
        topology = {}
        nbrDistance = {}
        for i in range(nodeNum):
            topology[self.nodes[i]] = []
            nbrDistance[self.nodes[i]] = []
            for j in range(nodeNum):
                if self.DistanceMatrix[i][j] <= self.COMMRANGE and i != j:
                    # 根据距离大小决定插入Topology的顺序
                    if len(topology[self.nodes[i]]) == 0:
                        topology[self.nodes[i]].append(self.nodes[j])
                        nbrDistance[self.nodes[i]].append(self.DistanceMatrix[i][j])
                    else:
                        for k in range(len(topology[self.nodes[i]])):
                            if self.DistanceMatrix[i][j] < nbrDistance[self.nodes[i]][k]:
                                topology[self.nodes[i]].insert(k,self.nodes[j])
                                nbrDistance[self.nodes[i]].insert(k,self.DistanceMatrix[i][j])
                                break
                            if k == len(topology[self.nodes[i]]) - 1:
                                topology[self.nodes[i]].append(self.nodes[j])
                                nbrDistance[self.nodes[i]].append(self.DistanceMatrix[i][j])
        return topology,nbrDistance

    def getNodeTopology(self, NodeId):
        """
        获取某个节点的邻居拓扑
        """
        i = self.nodes.index(NodeId)
        nodeNum = len(self.nodes)
        nodeTopology = []
        nodeNbrDistance = []
        for j in range(nodeNum):
            if self.DistanceMatrix[i][j] <= self.COMMRANGE and i != j:
                # 根据距离大小决定插入Topology的顺序
                if len(nodeTopology) == 0:
                    nodeTopology.append(self.nodes[j])
                    nodeNbrDistance.append(self.DistanceMatrix[i][j])
                else:
                    for k in range(len(nodeTopology)):
                        if self.DistanceMatrix[i][j] < nodeNbrDistance[k]:
                            nodeTopology.insert(k,self.nodes[j])
                            nodeNbrDistance.insert(k,self.DistanceMatrix[i][j])
                            break
                        if k == len(nodeTopology) - 1:
                            nodeTopology.append(self.nodes[j])
                            nodeNbrDistance.append(self.DistanceMatrix[i][j])  
        return nodeTopology,nodeNbrDistance


    # 两点坐标之间的距离
    def coordDistance(self, location1,location2):
        # location = {
        #          "X": 0, "Y": 6, "Z": -2
        #          },
        location1 = np.array([location1["X"], location1["Y"], location1["Z"]])
        location2 = np.array([location2["X"], location2["Y"], location2["Z"]])
        distance = np.linalg.norm(location1 - location2)
        distance = round(distance,3)
        return distance

    def geoDistance(self, location1,location2):
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
    
if __name__ == "__main__":

    daspMap = DaspMap()
    daspMap.run()
    daspMap.wait()

    # # 1.获取节点之间的拓扑结构
    # topology,nbrDistance= daspMap.getTopology()
    # name = "1"
    # print(topology[name],nbrDistance[name])

    # # 2.更新节点位置
    # daspMap.updateDistanceMatrix("1",{"X": 5, "Y": 10, "Z": -2})
    # # topology,nbrDistance= daspMap.getTopology()
    # nodeTopology,nodeNbrDistance = daspMap.getNodeTopology(name)

    # print(nodeTopology,nodeNbrDistance)