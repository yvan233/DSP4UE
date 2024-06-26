import csv
import json
import os
import platform
import select
import socket
import struct
import threading
import time
import traceback
from datetime import datetime
from . import DaspCommon, TcpSocket, Task, Const

class BaseServer(DaspCommon):
    '''基础服务器

    Dsp基础服务器

    属性:
        TaskDict: 任务字典
    '''
    TaskDict = {}

    def __init__(self):
        pass

    def recv_short_conn(self, host, port):
        '''
        短连接循环接收数据框架
        '''   
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind((host, port))
        server.listen(100) #接收的连接数
        while True:
            conn, addr = server.accept()
            # print('Connected by', addr)
            headPack,body = TcpSocket.recv(conn)
            self.handleMessage(headPack,body,conn)
            conn.close()
               
    def recv_long_conn(self, server, nbrID = ""):
        """
        长连接循环接收数据框架
        """
        server.listen(1) #接收的连接数
        while True:
            conn, addr = server.accept()
            print('Connected by {}:{}'.format(addr[0], addr[1]))
            conn.setblocking(False)
            # FIFO消息队列
            dataBuffer = bytes()
            with conn:
                while True:
                    ready_to_read, _, _ = select.select([conn], [], [], Const.SYSTEM_TASK_TIME + Const.RECONNECT_TIME_INTERVAL)
                    if ready_to_read:
                        try:
                            data = conn.recv(1024)
                        except Exception as e:
                            # 发送端进程被杀掉
                            self.handleRecvDisconnection(addr, nbrID)
                            break
                        if data == b"":
                            # 发送端close()
                            self.handleRecvDisconnection(addr, nbrID)
                            break
                        if data:
                            # 把数据存入缓冲区，类似于push数据
                            dataBuffer += data
                            while True:
                                if len(dataBuffer) < self.headerSize:
                                    break  #数据包小于消息头部长度，跳出小循环
                                # 读取包头
                                headPack = struct.unpack(self.headformat, dataBuffer[:self.headerSize])
                                bodySize = headPack[1]
                                if len(dataBuffer) < self.headerSize+bodySize :
                                    break  #数据包不完整，跳出小循环
                                # 读取消息正文的内容
                                body = dataBuffer[self.headerSize:self.headerSize+bodySize]
                                body = body.decode()
                                body = json.loads(body)
                                # 数据处理
                                self.handleMessage(headPack, body, conn)
                                # 数据出列
                                dataBuffer = dataBuffer[self.headerSize+bodySize:] # 获取下一个数据包，类似于把数据pop出
                    else: #超时
                        print("socket timeout")
                        conn.close()
                        break

    def handleMessage(self, headPack, body, conn):
        """
        数据处理函数,子类可重构该函数
        """
        if headPack[0] == 1:
            print(body)
        else:
            print("非POST方法")

    def handleRecvDisconnection(self, addr, nbrID):
        """
        对接收数据时邻居断开连接的操作函数               
        """
        print ("{}:{} 已断开".format(addr[0],addr[1]))      

    def pingID(self,nbrID):
        """
        尝试通过TCP连接指定节点
        """
        data = {
            "key": "ping",
            "id": DaspCommon.nodeID
        }
        thread = threading.Thread(target=self.send_recv, args=(nbrID,data,))
        thread.start()
        # self.send_recv(nbrID, data)

    def pingIDWaited(self,nbrID):
        """
        尝试通过TCP连接指定节点，并等待
        """
        data = {
            "key": "ping",
            "id": DaspCommon.nodeID
        }
        self.send_recv(nbrID, data)

    def send(self,id,data):
        """
        通过TCP的形式将信息发送至指定ID的节点
        """
        if id not in DaspCommon.nbrSocket: 
            try:
                ip = DaspCommon.nodesIpDict[id]
                print (f"connecting to {id}")
                remote_ip = socket.gethostbyname(ip)
                assigned_port = self.connectNbrID(id)
                DaspCommon.nbrSocket[id] = TcpSocket(id,remote_ip,assigned_port,self)
            except:
                self.deleteTaskNbrID(id)
                return 0
        DaspCommon.nbrSocket[id].sendall(data)

    def connectNbrID(self, nodeID):
        '''
        请求连接邻居节点并获取端口号
        '''
        id = nodeID
        ip = DaspCommon.nodesIpDict[id]
        port = DaspCommon.nodesPortdict[id]
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ip, port))
        data = {
            "key": "connect",
            "id": DaspCommon.nodeID
        }
        methods = 1
        body = json.dumps(data)
        header = [methods, body.__len__()]
        headPack = struct.pack(self.headformat , *header)
        sock.sendall(headPack+body.encode())

        recv_data = sock.recv(1024)
        assigned_port = int(recv_data.decode())
        sock.close()
        return assigned_port

    def send_recv(self, id, data):
        """
        通过TCP的形式将信息发送至指定ID的节点,并等待返回结果
        """
        if id not in DaspCommon.nbrSocket: 
            try:
                ip = DaspCommon.nodesIpDict[id]
                print (f"connecting to {id}")
                remote_ip = socket.gethostbyname(ip)
                assigned_port = self.connectNbrID(id)
                DaspCommon.nbrSocket[id] = TcpSocket(id,remote_ip,assigned_port,self)
            except:
                self.deleteTaskNbrID(id)
                return 0

        DaspCommon.nbrSocket[id].sendall_recv(data)

    def forward2childID(self, jdata, DappName):
        """
        将json消息转发给子节点
        """
        if BaseServer.TaskDict[DappName].childID:
            for child in reversed(BaseServer.TaskDict[DappName].childID):
                self.send(child, data=jdata)

    def forward2parentID(self, jdata, DappName):
        """
        将json消息转发给父节点
        """
        if BaseServer.TaskDict[DappName].parentID != DaspCommon.nodeID:
            self.send(BaseServer.TaskDict[DappName].parentID, data=jdata)

    def deleteTaskNbrID(self, id):  
        """
        删除本节点的任务和指定id邻居节点的所有连接(任务字典中的变量)
        """
        self.deleteNbrID(id)
        for key in BaseServer.TaskDict:
            BaseServer.TaskDict[key].deleteTaskNbrID(id)       
        self.sendRunDatatoGUI(f"Disconnected with node {id}.") 
        
    def addTaskNbrID(self, id, direction):
        """
        添加本节点的任务和指定id邻居节点的所有连接(任务字典中的变量)
        """
        for key in BaseServer.TaskDict:
            BaseServer.TaskDict[key].addTaskNbrID(id, direction)    

    def sendRunDatatoGUI(self, info, DappName = "system"):
        """
        通过UDP的形式将运行信息发送至GUI
        """
        self.sendtoGUIbase(info, "RunData", DappName)

    def sendFlagtoGUI(self, info, DappName = "system"):
        """
        通过UDP的形式将运行状态发送至GUI  
        #0 未启动
        #1 暂停中
        #2 运行中
        """
        self.sendtoGUIbase(info, "RunFlag", DappName)

    def updateTopology(self, location, waited = True):
        """
        更新节点位置，维护节点之间的拓扑关系,可根据距离关系只连接最近的几个
        """
        # ['3', '6', '5', '9'] [5.0, 7.211, 9.22, 9.849]
        topology, nbrDistance = DaspCommon.mapSocket.getTopoFromMapSync(location)
        while set(topology) != set(DaspCommon.nbrID):
            # 删除不在拓扑中的节点
            for node in reversed(DaspCommon.nbrID):
                if node not in topology:
                    self.deleteTaskNbrID(node)
            # 添加新的节点
            for i,node in enumerate(topology):
                if node not in DaspCommon.nbrID:
                    DaspCommon.nbrID.append(node)
                    DaspCommon.addNbrIDFlag[node] = False
                    if DaspCommon.nbrDirection:
                        direction = max(DaspCommon.nbrDirection) + 1
                    else:
                        direction = 1
                    DaspCommon.nbrDirection.append(direction)
                    if waited:
                        self.pingIDWaited(node)
                    else:
                        self.pingID(node)
                    self.addTaskNbrID(node,direction)
                    # DaspCommon.wiredlessNbrID.append(node)
                    DaspCommon.addNbrIDFlag[node] = True
                    self.sendRunDatatoGUI(f"Connected with node {node}.")
            # if waited:
            while True:
                # 遍历所有节点，直到所有节点的addNbrIDFlag存在且都为True
                if set(DaspCommon.nbrID) == set(DaspCommon.addNbrIDFlag.keys()) and all(DaspCommon.addNbrIDFlag.values()):
                    break
                time.sleep(0.01)
        return topology, nbrDistance

class TaskServer(BaseServer):
    """外部交互服务器
    
    用于节点和外部交互
    
    属性:
        host: 绑定IP
        port: 绑定port
        ResultThreads: 结果转发多线程
    """
    def __init__(self,host,port):
        self.host = host
        self.port = port
        self.ResultThreadsFlag = 0

    def run(self):
        """
        服务器开始运行
        """
        print ("TaskServer on: {}:{}".format(self.host,str(self.port)))
        self.recv_short_conn(self.host, self.port)

    def handleMessage(self, headPack, body, conn):
        """
        数据处理函数,子类可重构该函数
        """
        try:
            jdata = body
            if headPack[0] == 1:
                if jdata["key"] == "connect":
                    self.respondConnect(jdata, conn)

                elif jdata["key"] == "newtask":
                    self.newtask(jdata)

                elif jdata["key"] == "pausetask":
                    self.pausetask(jdata)

                elif jdata["key"] == "resumetask":
                    self.resumetask(jdata)
                    
                elif jdata["key"] == "shutdowntask":
                    self.shutdowntask(jdata)

                else:
                    info = "您输入的任务信息有误！"
                    self.sendRunDatatoGUI(info+str(jdata))
                    print(info+str(jdata))
            else:
                info = "暂未提供POST以外的接口"
                self.sendRunDatatoGUI(info)
        except Exception as e:
            self.sendRunDatatoGUI("Task server error!")
            print(traceback.format_exc())
            self.sendRunDatatoGUI(traceback.format_exc())

    def respondConnect(self, jdata, conn):
        """
        建立邻居节点连接，分配通信服务器端口号
        """
        id = jdata["id"]
        if id not in DaspCommon.assignedPortDict:
            commserver = CommServer(DaspCommon.IP)
            t = threading.Thread(target=commserver.run,args=())
            t.setDaemon(True)
            t.start()
            DaspCommon.commServerThread.append(t)
            DaspCommon.assignedPortDict[id] = commserver.get_bound_port()
        conn.sendall(str(DaspCommon.assignedPortDict[id]).encode())

        #动态加入邻居节点，暂时删除、避免异步导致的问题，现有拓扑已是双向，无需再次添加
        # if id not in DaspCommon.nbrID:
        #     DaspCommon.nbrID.append(id)
        #     DaspCommon.addNbrIDFlag[id] = False
        #     if DaspCommon.nbrDirection:
        #         direction = max(DaspCommon.nbrDirection) + 1
        #     else:
        #         direction = 1
        #     DaspCommon.nbrDirection.append(direction)
        #     self.pingID(id)
        #     self.sendRunDatatoGUI(f"Connected with node {id}")
        #     self.addTaskNbrID(id,direction)
        #     # 放在最后以标志连接成功
        #     DaspCommon.addNbrIDFlag[id] = True

    def newtask(self, jdata): 
        """
        启动DAPP，加载任务、建立通信树、启动任务、启动数据转发线程
        """
        name = jdata["DappName"]
        self.sendRunDatatoGUI("Receive new task requests.",name)
        while not DaspCommon.systemFlag: time.sleep(0.01)
        task = Task(name, self)
        BaseServer.TaskDict[name] = task
        task.load()
        task.startCommPattern()

    def pausetask(self, jdata):
        """
        暂停DAPP
        """
        name = (jdata["DappName"])
        self.forward2childID(jdata, name)
        BaseServer.TaskDict[name].pause()
        self.sendFlagtoGUI(1,name)

    def resumetask(self, jdata):
        """
        恢复DAPP
        """
        name = (jdata["DappName"])
        self.forward2childID(jdata, name)
        BaseServer.TaskDict[name].resume()
        self.sendFlagtoGUI(2,name)

    def shutdowntask(self, jdata):
        """
        停止DAPP
        """
        name = jdata["DappName"]
        self.forward2childID(jdata,name)
        BaseServer.TaskDict[name].shutdown()
        self.sendFlagtoGUI(0,name)

    def systemtask(self):
        """
        系统任务，不断ping邻居节点，维护拓扑
        """
        DaspCommon.systemFlag = False
        self.sendRunDatatoGUI("{} system start.".format(DaspCommon.nodeID))
        if platform.system() == "Linux":
            time.sleep(5)  # wait for other nodes to start
        while(True):
            last_time = time.time()
            for ele in reversed(DaspCommon.nbrID):
                self.pingID(ele)

            if DaspCommon.systemFlag == False:  
                # 第一轮启动更新无线拓扑
                if DaspCommon.wiredless:
                    self.updateTopology(DaspCommon.location, waited = False)
                # 第一轮开启系统自启动任务进程
                self.startthreads = threading.Thread(target=self.autostarttask, args=())
                self.startthreads.start()
            
            DaspCommon.systemFlag = True
            self.sendRunDatatoGUI("The current neighbors are {}".format(str(DaspCommon.nbrID)))
            if time.time() - last_time < Const.SYSTEM_TASK_TIME:
                time.sleep(Const.SYSTEM_TASK_TIME - time.time() + last_time)
     
    def autostarttask(self):
        """
        启动开机自启动任务
        """
        path = os.getcwd() + "/Dapp/Base/autostart.csv"
        with open(path,'r',encoding='utf-8-sig')as f:
            data = csv.reader(f)
            dapp_autostart = []
            for i in data:
                dapp_autostart.append(i)
        del dapp_autostart[0]        
        # 依据time排序
        dapp_autostart = sorted(dapp_autostart,key=(lambda x:x[1]))
        beforetime = 0
        for ele in dapp_autostart:
            name = ele[0]
            time.sleep(float(ele[1]) - beforetime)
            # self.sendRunDatatoGUI(f"Prepare to start autostart task: {name}.")
            if name not in BaseServer.TaskDict:
                task = Task(name, self)
                BaseServer.TaskDict[name] = task
                task.load()
                task.startCommPattern()
            beforetime = float(ele[1])

class CommServer(BaseServer):
    """
    通信服务器，用于节点和其他节点通信
    """
    host = "locolhost"

    def __init__(self,host):
        self.host = host
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((host, 0))
        self.bound_port = self.server.getsockname()[1] # 获取绑定的端口号
        

    def get_bound_port(self):
        return self.bound_port

    def run(self):
        """
        服务器开始运行
        """
        print ("CommServer on: {}:{}".format(self.host,self.bound_port))
        self.recv_long_conn(self.server)

    def handleMessage(self, headPack, body, conn):
        """
        数据处理函数
        """
        try:
            jdata = body
            if headPack[0] == 1:
                #建立通信树
                if jdata["key"] == "ping":
                    self.respondPing(jdata, conn)

                elif jdata["key"] == "shutdowntask":
                    self.respondShutDownTask(jdata)

                elif jdata["key"] == "pausetask":
                    self.respondPauseTask(jdata)
                
                elif jdata["key"] == "resumetask":
                    self.respondResumeTask(jdata)

                elif jdata["key"] == "data":
                    self.respondData(jdata)

                elif jdata["key"] == "questionData":
                    self.respondQuestionData(jdata)
                    
                elif jdata["key"] == "AsynchData":
                    self.respondAsynchData(jdata)

                elif jdata["key"] == "AlstData":
                    self.respondAlstData(jdata)

                elif jdata["key"] == "RootData":
                    self.respondRootData(jdata)

                elif jdata["key"] == "DescendantData":
                    self.respondDescendantData(jdata)

                elif jdata["key"] == "sync":
                    self.respondSync(jdata,1)

                elif jdata["key"] == "sync2":
                    self.respondSync(jdata,2)

                else:
                    info = "请不要直接访问通信服务器"
                    self.sendRunDatatoGUI(info)

            else:
                info = "非POST方法，请不要直接访问通信服务器"
                self.sendRunDatatoGUI(info)
                
        except Exception as e:
            self.sendRunDatatoGUI("Communication server error!")
            print(traceback.format_exc())
            self.sendRunDatatoGUI(traceback.format_exc())

    def respondPing(self, jdata, conn):
        """
        回应ping信号，如果发送的节点之前不在邻居节点中则加入网络
        """
        ID = jdata["id"]
        conn.sendall(b'pong')
        
    def respondPauseTask(self, jdata):
        """
        回应暂停任务信号，广播子节点暂停任务信号，暂停任务DAPP
        """
        name = (jdata["DappName"])
        self.forward2childID(jdata, name)
        BaseServer.TaskDict[name].pause()

    def respondResumeTask(self, jdata):
        """
        回应恢复任务信号，广播子节点恢复任务信号，恢复任务DAPP
        """
        name = (jdata["DappName"])
        self.forward2childID(jdata, name)
        BaseServer.TaskDict[name].resume()

    def respondShutDownTask(self, jdata):
        """
        回应结束任务信号，广播子节点结束任务信号，结束任务DAPP
        """
        name = (jdata["DappName"])
        self.forward2childID(jdata, name)
        BaseServer.TaskDict[name].shutdown()

    def respondData(self, jdata):
        """
        回应子节点任务结束信号，并收集数据
        """
        name = jdata["DappName"]
        index = BaseServer.TaskDict[name].childID.index(jdata["id"])
        BaseServer.TaskDict[name].childData[index] = jdata["data"]
        if all(BaseServer.TaskDict[name].childData):
            BaseServer.TaskDict[name].childDataEndFlag = 1

    def respondQuestionData(self, jdata):
        """
        回应任务发送数据信号，并存储数据
        """
        name = jdata["DappName"]
        while(jdata["id"] not in BaseServer.TaskDict[name].taskNbrID): time.sleep(0.01)
        index = BaseServer.TaskDict[name].taskNbrID.index(jdata["id"])
        if jdata["type"] == "value":
            BaseServer.TaskDict[name].nbrData[index] = jdata["data"]

        elif jdata["type"] == "value2":
            BaseServer.TaskDict[name].nbrData2[index] = jdata["data"]

    def respondAlstData(self, jdata):
        """
        回应任务发送数据信号，并存储数据
        """
        name = jdata["DappName"]
        while not DaspCommon.systemFlag: time.sleep(0.01)
        if name not in BaseServer.TaskDict:
            task = Task(name, self)
            BaseServer.TaskDict[name] = task
            task.load()
            task.startCommPattern()
        elif BaseServer.TaskDict[name].commTreeFlag == 0:
            task = Task(name, self)
            BaseServer.TaskDict[name] = task
            task.load()
            task.startCommPattern()
        task = BaseServer.TaskDict[name]
        while(not hasattr(task,'loadflag')):time.sleep(0.01)
        while(task.loadflag == 0):time.sleep(0.01)
        if jdata["id"] in task.taskNbrID:
            index = task.taskNbrID.index(jdata["id"])
            task.nbrAlstData[index].put(jdata["data"])


    def respondAsynchData(self, jdata):
        """
        回应任务发送数据信号，并存储数据
        """
        name = jdata["DappName"]
        task = BaseServer.TaskDict[name]
        index = task.taskNbrID.index(jdata["id"])
        task.nbrAsynchData[index].put(jdata["data"])
            
    def respondRootData(self, jdata):
        """回应任务发送数据至根节点信号

        返回值：
            data: 后代节点数据
            path: 后代节点发送过来的路径
            recvtime: 接受到消息的时刻
        """
        task_cur = BaseServer.TaskDict[jdata["DappName"]]
        # 如果本节点是根节点则存储数据
        if task_cur.parentID == DaspCommon.nodeID:
            # 加入接收消息的时间，方便时间同步
            task_cur.descendantData.put([jdata["data"], jdata["path"], datetime.now()])
        # 否则将数据转发给父节点
        else:  
            jdata["path"].append(DaspCommon.nodeID)
            self.send(task_cur.parentID, jdata)

    def respondDescendantData(self, jdata):
        """回应任务发送数据至后代节点信号

        返回值：
            data: 根节点数据
            recvtime: 接受到消息的时刻
        """
        task_cur = BaseServer.TaskDict[jdata["DappName"]]
        # 如果指定了路径
        if jdata["path"] != None: 
            if jdata["path"]:
                nextnode = jdata["path"].pop()
                self.send(nextnode, jdata)
            # 如果目标是本节点
            else:
                task_cur.rootData.put([jdata["data"], datetime.now()])                
        # 否则进行广播
        else:
            self.forward2childID(jdata, jdata["DappName"])
            task_cur.rootData.put([jdata["data"], datetime.now()])

    def respondSync(self, jdata, type):
        """
        回应同步请求，并改变相应标志位
        """
        name = jdata["DappName"]
        while(name not in BaseServer.TaskDict):time.sleep(0.01)
        while(not hasattr(BaseServer.TaskDict[name],'loadflag')):time.sleep(0.01)
        while(BaseServer.TaskDict[name].loadflag == 0):time.sleep(0.01)
        while(jdata["id"] not in BaseServer.TaskDict[name].taskNbrID): time.sleep(0.01)
        index = BaseServer.TaskDict[name].taskNbrID.index(jdata["id"])
        if type == 1:
            BaseServer.TaskDict[name].nbrSyncStatus[index] = 1
        else:
            BaseServer.TaskDict[name].nbrSyncStatus2[index] = 1