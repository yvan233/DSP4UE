import codecs
import copy
import importlib
import json
import os
import queue
import socket
import threading
import time
import traceback

from ..pysnooperdb.tracer import Tracer as snoop
from . import DaspCommon

class Task(DaspCommon):
    """任务类
    
    用于任务具体计算
    
    属性:
        nodeID: 节点ID
        nbrID:邻居ID
        leader: 领导节点(根节点)
        childID: 子节点ID
        parentID: 父节点ID,根节点与nodeID相同
        parentDirection 父节点方向,根节点为-1
        childDirection  子节点方向
        taskRouteTable: 邻居IP及端口列表
        nbrData: 邻居数据
        nbrData2: 同步通信函数中另一个邻居变量
        nbrDirection: 邻居方向
        nbrAsynchData: 异步邻居数据
        nbrAlstData: 异步邻居数据(Alst)
        childData: 子节点数据
        rootData: 根节点数据(队列)
        descendantData: 后代节点数据(队列)

        DappName: 任务名称
        GuiInfo: UI界面IP及端口
        taskfunc: 从算法文件中加载的算法程序
        taskNbrDirection: 当前任务邻居方向
        taskDatalist: 当前任务初始数据
        taskNbrID:当前任务邻居ID
        taskID: 当前任务参与的节点ID
        resultInfo: 运行结果数据
        resultInfoQue: 运行结果数据队列
        taskThreads :多线程

        commTreeFlag: 当前节点是否被加进通信树标志
        treeWaitFlag: 生成树维护等待标志
        taskBeginFlag: 任务启动标志
        taskEndFlag: 任务结束标志
        dataEndFlag: 根节点数据收集结束标志
        childDataEndFlag: 子节点数据收集结束标志   
        runFlag: 用于暂停的标志，threading.Event()
            .set()设置为True
            .clear()设置为False
            .wait()  当为True时pass，当为False时阻塞等到到为True时pass
        timeFlag: 用于timesleep

        syncTurnFlag: 同步函数轮训标志
        syncTurnFlag2: 同步通信函数轮训标志
        nbrSyncStatus: 同步函数邻居状态
        nbrSyncStatus2: 同步函数邻居状态2(轮训)    

        debugMode: 调试模式是否启用标志
        databaseInfo: 数据库连接信息
        databaseName: 调试数据库名称（默认Daspdb）
        observedVariable: 观察变量列表

        deleteDirection: 删除的节点方向
        """
    def __init__(self, DappName, owner):
        self.DappName = DappName
        self.owner = owner
        self.commTreeFlag = 1
        self.treeWaitFlag = 0
        self.dataEndFlag = 0
        self.loadflag = 0 
        self.timesleepFlag = threading.Event()
        self.runFlag = threading.Event()
        self.runFlag.set()
        
    def load(self):
        """
        加载任务运行所需信息,启动run线程,等待任务启动标志
        """
        # 加载topology文件
        ID = []
        AllwiredNbrID = []
        AllDatalist = []
        AllWiredless = []
        # AllNbrDirection = []
        path = f"{os.getcwd()}/Dapp/{self.DappName}/topology.json"
        # 如果没有就用基本拓扑
        if not os.path.exists(path):
            path = os.getcwd() + "/Dapp/Base/topology.json"
        text = codecs.open(path, 'r', 'utf-8').read()
        js = json.loads(text)
        for ele in js["Nodes"]:
            if "ID" in ele:
                ID.append(ele["ID"])
                AllwiredNbrID.append(ele["WiredNbrID"])
                # AllNbrDirection.append(ele["nbrDirection"])
                AllDatalist.append(ele["Data"])
                AllWiredless.append(ele["Wireless"])
        self.taskID = ID
        
        ## 如果当前节点在任务中
        if DaspCommon.nodeID in self.taskID:
            # 加载question文件
            try:
                question = importlib.import_module(f"Dapp.{self.DappName}.question")
                question = importlib.reload(question)
                self.taskfunc = question.taskFunction
            except Exception as e:
                self.sendDatatoGUI("DAPP import error!")
                print(traceback.format_exc())
                self.sendDatatoGUI(traceback.format_exc())

            # 加载debug信息
            debugpath = f"{os.getcwd()}/Dapp/{self.DappName}/debug.json"
            self.loadDebugInfo(debugpath)

            order = ID.index(DaspCommon.nodeID)
            selfWiredNbrID = AllwiredNbrID[order]
            # selfNbrDirection = AllNbrDirection[order]
            selfNbrDirection = [ i+1 for i in range(len(selfWiredNbrID))]
            selfDatalist = AllDatalist[order]
            selfwiredless = AllWiredless[order]
            # selfRouteTable = []

            ### 如果节点已经掉线了，那初始化的任务节点ID不包含这个节点
            for i in range(len(selfWiredNbrID)-1,-1,-1):
                if selfWiredNbrID[i] not in DaspCommon.nbrID:
                    selfWiredNbrID.remove(selfWiredNbrID[i])
                    selfNbrDirection.remove(selfNbrDirection[i])

            self.taskInitWiredNbrID = copy.deepcopy(AllwiredNbrID[order])
            self.taskNbrID = selfWiredNbrID
            self.taskNbrDirection = selfNbrDirection
            self.taskDatalist = selfDatalist
            self.taskwiredless = selfwiredless

            ## 添加无线邻居节点
            if self.taskwiredless:
                for ele in DaspCommon.nbrID:
                    # 在任务节点中,但尚未在邻居节点中
                    if (ele in self.taskID) and (ele not in self.taskNbrID):
                        self.taskNbrID.append(ele)
                        if selfNbrDirection:
                            direction = max(selfNbrDirection) + 1
                        else:
                            direction = 1
                        self.taskNbrDirection.append(direction)

            # 初始化各属性
            self.taskBeginFlag = 0
            self.taskEndFlag = 0
            self.dataEndFlag = 0
            self.resultInfo = {}
            self.resultInfoQue = []
            self.childData = []
            self.childDataEndFlag = 0
            self.parentID = DaspCommon.nodeID
            self.parentDirection = -1
            self.leader = None
            
            self.childID= []
            self.childDirection = []
            self.nbrData = []
            self.nbrAsynchData = []
            self.nbrAlstData = []
            self.nbrData2= []
            self.rootData = queue.Queue() 
            self.descendantData = queue.Queue() 
            self.deleteDirection = queue.Queue()

            self.syncTurnFlag = 0
            self.syncTurnFlag2 = 0
            self.nbrSyncStatus = []
            self.nbrSyncStatus2 = []

            while len(self.nbrData) < len(self.taskNbrDirection):
                self.nbrData.append([])
                self.nbrAsynchData.append(queue.Queue())
                self.nbrAlstData.append(queue.Queue())
                self.nbrData2.append([])
                self.nbrSyncStatus.append(0)
                self.nbrSyncStatus2.append(0)

            self.taskThreads = threading.Thread(target=self.run, args=())
            self.taskThreads.start()
        self.loadflag = 1 

    def loadDebugInfo(self, debugpath):
        """
        加载调试模式信息
        """
        self.databaseName = 'Daspdb'
        if not os.path.exists(debugpath):
            self.debugMode = False
            self.databaseInfo = []
            self.observedVariable = []
        else:
            text = codecs.open(debugpath, 'r', 'utf-8').read()
            jdata = json.loads(text)
            self.debugMode = jdata["debugMode"]
            self.databaseInfo = jdata["databaseInfo"]
            self.observedVariable = jdata["observedVariable"]

    def run(self):
        """
        任务服务器开始运行,等待任务启动标志
        """
        while 1:
            time.sleep(0.01)
            if self.taskBeginFlag == 1:
                try:
                    if self.debugMode:
                        tablename = "{}_{}".format(self.DappName, self.nodeID)
                        taskfunc = snoop(config = self.databaseInfo, db = self.databaseName, tablename = tablename, \
                            observelist = self.observedVariable)(self.taskfunc)
                        self.sendDatatoGUI("start task in debug mode")
                    else:
                        taskfunc = self.taskfunc
                        self.sendDatatoGUI("start task")
                    print("DAPP:{} start".format(self.DappName))
                    value = taskfunc(self, DaspCommon.nodeID, self.taskNbrDirection, self.taskDatalist)
                    self.resultInfo["value"] = value
                    self.sendDatatoGUI("complete task")
                    # time.sleep(1)  # 防止该节点任务结束，其他节点的同步函数出错
                    print ("Calculation complete")
                    self.taskBeginFlag = 0
                    self.taskEndFlag = 1
                    self.collecResult()
                except SystemExit as e:
                    self.sendDatatoGUI("stop task")
                    print("DAPP:{} stop".format(self.DappName))
                except Exception as e:
                    self.sendDatatoGUI("task error!")
                    print(traceback.format_exc())
                    self.sendDatatoGUI(traceback.format_exc())
                finally:
                    self.reset()
                return 0

    def pause(self):
        """
        暂停运行任务服务器
        """
        self.runFlag.clear()     # 设置为False, 阻塞
        self.sendtoGUIbase("pause", "RunData", self.DappName) # 防止阻塞，不用sendDatatoGUI
        print("DAPP:{} pause".format(self.DappName))

    def resume(self):
        """
        恢复运行任务服务器
        """
        self.runFlag.set()    # 设置为True, 停止阻塞
        self.sendtoGUIbase("resume", "RunData", self.DappName) # 防止阻塞，不用sendDatatoGUI
        print("DAPP:{} resume".format(self.DappName))

    def shutdown(self):
        """
        终止运行任务服务器
        """
        try:
            # 停止所有timesleep
            self.timesleepFlag.set()
            self.stop_thread(self.taskThreads)
        # 此时任务线程已退出
        except ValueError:
            self.sendDatatoGUI("stop")
            print("DAPP:{} has stopped".format(self.DappName))
        self.reset()

    def collecResult(self):
        """
        收集算法程序运行结果
        """
        # 叶子结点
        self.syncNode() #收集数据前同步一次，防止父节点算法先计算结束，等待子节点超时
        if self.childID == []:
            # 根节点
            if self.parentID == DaspCommon.nodeID:
                sdata = {
                    "id": DaspCommon.nodeID,
                    "info": self.resultInfo
                }
                self.resultInfoQue.append(sdata)
                self.dataEndFlag = 1
                print ("The whole data has been transmitted!")
            else:
                sdata = {
                    "id": DaspCommon.nodeID,
                    "info": self.resultInfo
                }
                self.resultInfoQue.append(sdata)
                data = {
                    "key": "data",
                    "DappName": self.DappName,
                    "id": DaspCommon.nodeID,
                    "data": self.resultInfoQue
                }
                self.send(self.parentID, data)

        # 非叶子结点
        else:
            print ("wait childDataEndFlag")
            while self.childDataEndFlag == 0:
                time.sleep(0.01)
            print ("childDataEndFlag complete")
            sdata = {
                "id": DaspCommon.nodeID,
                "info": self.resultInfo
            }
            self.resultInfoQue.append(sdata)
            for ele in self.childData:
                for ele2 in ele:
                    self.resultInfoQue.append(ele2)
            if self.parentID != DaspCommon.nodeID:
                data = {
                    "key": "data",
                    "DappName": self.DappName,
                    "id": DaspCommon.nodeID,
                    "data": self.resultInfoQue
                }
                self.send(self.parentID, data=data)
                self.resultInfoQue = []
            else:
                self.dataEndFlag = 1
                print ("The whole data has been transmitted!")

    def reset(self):
        """
        重置当前节点
        """
        self.commTreeFlag = 0
    
    def resetNbrData(self):
        """
        清空邻居数据
        """
        for i in range(len(self.nbrData)):
            self.nbrAsynchData[i] = []
            self.nbrAlstData[i] = queue.Queue()
            self.nbrData[i] = []
            self.nbrData2[i] = []

    def send(self,id,data):
        """
        通过TCP的形式将信息发送至指定ID的节点
        """
        self.runFlag.wait()
        self.owner.send(id,data)

    def sendData(self, data):
        """
        通过TCP的形式将信息发送至所有邻居
        """
        for ele in reversed(self.taskNbrID):
            thread = threading.Thread(target=self.send, args=(ele,data,))
            thread.start()
            
    def deleteTaskNbrID(self, id):  
        """
        删除本节点和指定id邻居节点的所有连接
        """
        if id in self.taskNbrID:
            index = self.taskNbrID.index(id)    
            direction = self.taskNbrDirection[index]
            del self.taskNbrID[index]
            del self.taskNbrDirection[index]   
            del self.nbrSyncStatus[index] 
            del self.nbrSyncStatus2[index]     
            del self.nbrData[index]
            del self.nbrData2[index]
            del self.nbrAsynchData[index]
            del self.nbrAlstData[index]
            self.deleteDirection.put(direction) 


    def addTaskNbrID(self, id, direction):
        """
        添加本节点和指定id邻居节点的连接
        """
        # 节点在任务中
        if id in self.taskID:
            # 节点在初始有线邻居节点中 或 节点本身是无线的
            if id in self.taskInitWiredNbrID or self.taskwiredless == 1:
                self.taskNbrDirection.append(direction)
                self.nbrSyncStatus.append(0)
                self.nbrSyncStatus2.append(0)
                self.nbrData.append([])
                self.nbrData2.append([])
                self.nbrAsynchData.append(queue.Queue())
                self.nbrAlstData.append(queue.Queue())
                self.taskNbrID.append(id)

    def startCommPattern(self):
        """
        start comm pattern
        """
        self.commThread = threading.Thread(target=self.commPattern, args=(None, True,False,))
        self.commThread.start()

    def floodLeaderElection(self):
        """
        领导人选举
        """
        self.leader = None
        min_id = DaspCommon.nodeID
        for m in range(len(self.taskID)-1):  # 迭代节点数量的轮次数
            _, nbrData = self.transmitData(self.taskNbrDirection, [min_id]*len(self.taskNbrDirection))      
            for ele in nbrData:
                if ele < min_id:
                    min_id = ele
        self.leader = min_id
        if self.leader == DaspCommon.nodeID:
            self.sendDatatoGUI("This is the leader")

    def commPattern(self, minValue = None, InitFlag = False, MaintainFlag = False):
        """
        communication pattern, minValue排序依据的value, Initflag:是否初始化，MaintainFlag:是否维护
        """
        def reset():
            flag = False
            parent = -1
            child = []    
            edgesFlag = [False] * len(nbrDirection)
            edges = dict(zip(nbrDirection,edgesFlag))
            return flag,parent,child,edges
        def alst(nbrDirection,value,flag,parent,child,edges,min_value,j,data,token):
            """
            Asynchronous Leaderless Spanning Tree algorithm 
            """
            leader_state = "non-leader"
            while True:
                if token <= min_value:
                    if token < min_value:
                        min_value = token
                        flag,parent,child,edges = reset()
                    edges[j] = True
                    if data == "end" and j == parent:
                        for ele in child:
                            self.sendAlstData(ele,["end",min_value]) 
                        break
                    elif data == "join":
                        if j not in child:
                            child.append(j)
                    elif data == "search":
                        if flag == False:
                            flag = True
                            parent = j 
                            for ele in nbrDirection:
                                if ele != j:
                                    self.sendAlstData(ele,["search",min_value])
                    if all(edges.values()):
                        self.leader = min_value
                        if min_value == value:
                            leader_state = "leader"
                            for ele in child:
                                self.sendAlstData(ele,["end",min_value]) 
                            break
                        else:
                            leader_state = "non-leader"  
                            self.sendAlstData(parent,["join",min_value]) 
                j,(data,token) = self.getAlstData()
                # self.sendDatatoGUI(f"j:{j},data:{data},token:{token}")
            return leader_state,parent,child
        def setTree(parent,child):
            """
            set parent,child,etc.
            """
            self.childDirection = child
            self.parentDirection = parent
            self.childID = [nbrID[nbrDirection.index(ele)] for ele in child]
            self.parentID = nbrID[nbrDirection.index(parent)] if parent != -1 else nodeID
            self.childData = [[]]*len(child)
    
        while self.treeWaitFlag: # 等待上一次生成树维护结束
            time.sleep(0.01)
        self.treeWaitFlag = True
        nbrDirection = self.taskNbrDirection
        nbrID = self.taskNbrID
        nodeID = DaspCommon.nodeID
        if minValue != None:
            value = minValue
        else:
            value = copy.deepcopy(DaspCommon.nodeID)  # 如果没有value传入，那么默认为节点ID
        flag,parent,child,edges = reset()
        min_value = value
        flag = True
        step = 1
        if nbrDirection:
            for ele in nbrDirection:
                self.sendAlstData(ele,["search",min_value])
            j,(data,token) = self.getAlstData()
            # self.sendDatatoGUI(f"j:{j},data:{data},token:{token}")
            while True:
                if step == 1:
                    __,parent,child = alst(nbrDirection,value,flag,parent,child,edges,min_value,j,data,token)
                    step = 2
                    # generate complete
                    setTree(parent,child)
                    # self.sendDatatoGUI("parentID:{},childID:{}".format(self.parentID,self.childID))
                    if InitFlag:
                        self.starttask()
                if step == 2:
                    if not MaintainFlag:
                        self.treeWaitFlag = False
                        break
                    if not self.deleteDirection.empty():
                        while not self.deleteDirection.empty():
                            direction = self.deleteDirection.get_nowait()
                            if self.parentDirection == direction:
                                self.parentID = DaspCommon.nodeID
                                self.parentDirection = -1
                                flag,parent,child,edges = reset()
                                min_value = value
                                flag = True
                                step = 1
                                for ele in nbrDirection:
                                    self.sendAlstData(ele,["search",min_value])
                                j,(data,token) = self.getAlstData()
                            elif direction in self.childDirection:
                                index = self.childDirection.index(direction) 
                                del edges[direction]
                                del self.childID[index]     
                                del self.childDirection[index]
                                del self.childData[index]
                            else:
                                # 处理一些异步的问题
                                if direction in edges:
                                    del edges[direction]
                    else:
                        for i,que in enumerate(self.nbrAlstData):
                            if not que.empty():
                                qdata = que.get_nowait()
                                j,(data,token) = nbrDirection[i],qdata
                                if j == self.parentDirection:
                                    self.parentID = DaspCommon.nodeID
                                    self.parentDirection = -1
                                    flag,parent,child,edges = reset()
                                    min_value = value
                                    flag = True
                                    step = 1
                                    if token > min_value :
                                        for ele in nbrDirection:
                                            self.sendAlstData(ele,["search",min_value])
                                else:
                                    if j in self.childDirection:
                                        index = self.childDirection.index(j) 
                                        del self.childID[index]
                                        del self.childDirection[index]
                                        del self.childData[index]
                                    edges[j] = False
                                    # Only the leader node can directly maintain the spanning tree
                                    if self.leader == nodeID:                                    
                                        if token > self.leader:
                                            self.sendAlstData(j,["search",self.leader])
                                        elif token < self.leader:
                                            step = 1
                                        else:
                                            edges[j] = True
                                            if data == "join":
                                                if j not in self.childDirection:
                                                    self.childDirection.append(j)
                                                    self.childID.append(nbrID[nbrDirection.index(j)])
                                                    self.childData.append([])
                                                self.sendAlstData(j,["end",self.leader]) 
                                            elif data == "search":
                                                self.sendAlstData(j,["end",self.leader]) 
                                    # The intermediate node enters the generation phase 
                                    # after receiving the alst message from its neighbor
                                    else:
                                        flag,parent,child,edges = reset()
                                        min_value = value
                                        flag = True
                                        step = 1
                                        if token > min_value:
                                            for ele in nbrDirection:
                                                self.sendAlstData(ele,["search",min_value])
                                break

                    time.sleep(0.01)
        else:
            self.starttask()
            self.treeWaitFlag = False
    
    def updateSpanningTree(self, minValue = None):
        """
        更新生成树
        """
        self.syncNode()
        self.commPattern(minValue = minValue, InitFlag = False, MaintainFlag = False)
        tree = {
            "childDirection": self.childDirection,
            "parentDirection": self.parentDirection,
            "childID": self.childID,
            "parentID": self.parentID
            }
        return tree

    def starttask(self):
        if self.parentDirection == -1:
            self.sendDatatoGUI("(Root node) The spanning tree has been built.")
            self.resultThread = threading.Thread(target=self.aggregateResult, args=())
            self.resultThread.start()
        self.taskBeginFlag = 1
            
                
    def aggregateResult(self):
        """
        根节点等待所有任务的数据收集结束标志，随后将计算结果转发到GUI界面
        """
        while 1:
            time.sleep(0.1)
            if self.dataEndFlag == 1:
                self.sendDatatoGUI("The task result aggregation completed.")
                info = []
                content = ""
                Que = self.resultInfoQue
                for i in range(len(Que)):
                    if Que[i]["info"]:
                        info.append({})
                        info[-1]["ID"] = Que[i]["id"]
                        info[-1]["value"] = str(Que[i]["info"]["value"])
                
                infojson = json.dumps(info, indent=2)  #格式化输出，更便于查看
                content =  "Nodes number:{}\nInfo:{}\n\n".format(len(Que), infojson)
                self.sendEndDatatoGUI(content)
                self.taskEndFlag = 0
                self.dataEndFlag = 0
                self.resultInfoQue = []
                self.resultInfo = {}
                    
    ##################################
    ###   下面为提供给用户的接口函数   ###
    ##################################

    def forward2childID(self, data):
        """
        将json消息转发给子节点
        """
        if self.childID:
            for ele in reversed(self.childID):
                self.send(ele, data=data)

    def timesleep(self, timeout = 0):
        self.timesleepFlag.wait(timeout)

    def sendDatatoGUI(self, info):
        """
        通过UDP的形式将运行信息发送至GUI
        """
        self.runFlag.wait()  #系统运行标志
        self.sendtoGUIbase(info, "RunData", self.DappName)

    def sendEndDatatoGUI(self, info):
        """
        通过UDP的形式将结束信息发送至GUI
        """
        self.sendtoGUIbase(info, "EndData",self.DappName)

    def sendDataToID(self, id, data):
        """
        通过TCP的形式将信息发送至指定ID的邻居
        """
        data = {
            "key": "questionData",
            "type": "value",
            "DappName": self.DappName,
            "id": DaspCommon.nodeID,
            "data": data
        }
        self.send(id, data)

    def sendDataToRoot(self, data):
        """将消息发送至根节点

        通过邻居不断转发，同时将自己的id和路径上的id加进path
        """

        if self.parentID == DaspCommon.nodeID:
            err = "Error! This node is the root node!"
            self.sendDatatoGUI(err)
            return err
        else:
            data = {
                "key": "RootData",
                "DappName": self.DappName,
                "path": [DaspCommon.nodeID],
                "data": data
            }
            self.send(self.parentID, data)

    def sendDataToDescendant(self, data, path = None):
        """将消息发送至后代节点
        
        根据path将消息回馈到发送的节点，若path为空则进行广播
        path中目标节点在第0位，所以用pop的方法从后取出元素
        """
        data = {
            "key": "DescendantData",
            "DappName": self.DappName,
            "path": copy.deepcopy(path),
            "data": data
        }
        # 如果指定了路径
        if path != None:  
            if path:
                nextnode = data["path"].pop()
                self.send(nextnode, data)
            else:
                err = "Error! Please enter a valid path."
                self.sendDatatoGUI(err)
                return err
        # 否则进行广播
        else:
            for ele in self.childID:
                self.send(ele, data)


    def sendDataToDirection(self, direction, data):
        """
        通过TCP的形式将信息发送至指定方向的邻居
        """
        data = {
            "key": "questionData",
            "type": "value",
            "DappName": self.DappName,
            "id": DaspCommon.nodeID,
            "data": data
        }
        for i in range(len(self.taskNbrID)):
            if self.taskNbrDirection[i] == direction:
                self.send(self.taskNbrID[i], data)

    def sendDataToDirection2(self, direction, data):
        """
        通过TCP的形式将信息发送至指定方向的邻居，为了避免与sendDataToDirection在一个轮次中发生冲突
        """
        data = {
            "key": "questionData",
            "type": "value2",
            "DappName": self.DappName,
            "id": DaspCommon.nodeID,
            "data": data
        }
        for i in range(len(self.taskNbrID)):
            if self.taskNbrDirection[i] ==  direction:
                self.send(self.taskNbrID[i], data)

    def sendAsynchData(self, direction, data):
        """
        通过TCP的形式将信息发送至指定方向的邻居
        """
        data = {
            "key": "AsynchData",
            "DappName": self.DappName,
            "id": DaspCommon.nodeID,
            "data": data
        }
        for i in range(len(self.taskNbrID)):
            if self.taskNbrDirection[i] ==  direction:
                self.send(self.taskNbrID[i], data)

    def sendAlstData(self, direction, data):
        """
        通过TCP的形式将信息发送至指定方向的邻居
        """
        data = {
            "key": "AlstData",
            "DappName": self.DappName,
            "id": DaspCommon.nodeID,
            "data": data
        }
        for i in range(len(self.taskNbrID)):
            if self.taskNbrDirection[i] ==  direction:
                self.send(self.taskNbrID[i], data)

    def getAsynchData(self):
        """
        获取邻居发过来的数据
        """
        while True:
            for i,que in enumerate(self.nbrAsynchData):
                if not que.empty():
                    # 非阻塞性地获取数据
                    data = que.get_nowait()
                    return (self.taskNbrDirection[i],data)
            time.sleep(0.01)

    def getAlstData(self):
        """
        获取邻居发过来的alst数据
        """
        while True:
            for i,que in enumerate(self.nbrAlstData):
                if not que.empty():
                    data = que.get_nowait()
                    return (self.taskNbrDirection[i],data)
            time.sleep(0.01)

    def transmitData(self,direclist,datalist):
        """
        同步通信函数，将datalist中的数据分别发到direclist的邻居方向
        """
        if (self.syncTurnFlag2 == 0):
            self.syncTurnFlag2 = 1 - self.syncTurnFlag2
            for i in reversed(range(len(direclist))):
                self.sendDataToDirection(direclist[i],datalist[i])
            self.syncNode() 
            ans = copy.deepcopy([self.taskNbrDirection, self.nbrData])
            for i in range(len(self.nbrData)):
                self.nbrData[i] = []
        else:
            self.syncTurnFlag2 = 1 - self.syncTurnFlag2
            for i in reversed(range(len(direclist))):
                self.sendDataToDirection2(direclist[i],datalist[i])
            self.syncNode() 
            ans = copy.deepcopy([self.taskNbrDirection, self.nbrData2])
            for i in range(len(self.nbrData2)):
                self.nbrData2[i] = []    
        return ans

    def syncNode(self):
        """
        同步函数，所有节点同步一次
        """
        if self.syncTurnFlag == 0:   
            self.syncTurnFlag = 1 - self.syncTurnFlag
            data = {
                "key": "sync",
                "id": DaspCommon.nodeID,
                "DappName": self.DappName
            }  
            self.sendData(data)
            # 全部非空才能继续
            while (not all(self.nbrSyncStatus)):   
                time.sleep(0.01)
            for i in range(len(self.nbrSyncStatus)):
                self.nbrSyncStatus[i] = 0

        elif self.syncTurnFlag == 1:
            self.syncTurnFlag = 1 - self.syncTurnFlag
            data = {
                "key": "sync2",
                "id": DaspCommon.nodeID,
                "DappName": self.DappName
            }
            self.sendData(data)
            while (not all(self.nbrSyncStatus2)):   
                time.sleep(0.01)
            for i in range(len(self.nbrSyncStatus2)):
                self.nbrSyncStatus2[i] = 0

    def updateTopology(self, location):
        """
        更新节点位置，维护节点之间的拓扑关系
        """
        topology, nbrDistance = self.owner.updateTopology(location)
        return topology, nbrDistance