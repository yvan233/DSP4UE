import ctypes
import datetime
import inspect
import json
import platform
import select
import socket
import struct
import threading
import time

class Const():
    RECONNECT_FAIL_LIMIT = 3
    RECONNECT_TIME_INTERVAL = 30
    SYSTEM_TASK_TIME = 120



class MapSocket():
    def __init__(self, id, ip, port):
        self.ID = id
        self.remoteIP = ip
        self.remotePort = port

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        localIP = socket.gethostbyname(socket.gethostname())
        self.socket.bind((localIP, 0))
        self.localPort = self.socket.getsockname()[1 ]

    def getTopoFromMapSync(self, location):
        """
        通过UDP的形式将信息发送至mapserver,从而获取邻居拓扑,同步方式
        """
        try:
            addr = (self.remoteIP, self.remotePort)
            data = {
                "key": "GetTopologySync",
                "id": self.ID,
                "time": datetime.datetime.now().strftime("%m-%d %H:%M:%S"),
                "location": location
            }
            self.socket.sendto(json.dumps(data).encode('utf-8'), addr)
            data, __ = self.socket.recvfrom(1000000)
            jdata = json.loads(data)
            topology = jdata["topology"]
            nbrDistance = jdata["nbrDistance"]
            return topology, nbrDistance
        except Exception as e:
            print ("Failed to send" + location)
            return [],[]


    def getTopoFromMapAsync(self, location):
        """
        通过UDP的形式将信息发送至mapserver,从而获取邻居拓扑
        """
        try:
            addr = (DaspCommon.GuiInfo[0], DaspCommon.GuiInfo[2])
            data = {
                "key": "GetTopologyAsync",
                "id": DaspCommon.nodeID,
                "time": datetime.datetime.now().strftime("%m-%d %H:%M:%S"),
                "location": location
            }
            self.socket.sendto(json.dumps(data).encode('utf-8'), addr)
            data, __ = self.socket.recvfrom(1000000)
            jdata = json.loads(data)
            topology = jdata["topology"]
            nbrDistance = jdata["nbrDistance"]
            return topology, nbrDistance
        except Exception as e:
            print ("Failed to send" + location)
            return [],[]
        

class TcpSocket():
    headformat = "!2I"
    headerSize = 8
    def __init__(self, ID, ip, port, owner):
        self.ID = ID
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_keep_alive()
        self.sock.settimeout(5)
        self.sock.connect((self.ip, self.port))
        self.owner = owner

        # heartbeat
        self.fail_count = 0
        self.fail_limit = Const.RECONNECT_FAIL_LIMIT
        self.time_interval = Const.RECONNECT_TIME_INTERVAL
        self.state = "passing"

    def close(self):
        self.sock.close()

    def sendall(self, jsondata, methods = 1):
        '''
        为发送的json数据添加methods和length报头
            POST: methods = 1
            REFUSE: methods = 9
        '''
        body = json.dumps(jsondata)
        header = [methods, body.__len__()]
        headPack = struct.pack(TcpSocket.headformat , *header)
        try:
            self.sock.sendall(headPack+body.encode())
        except:
            self.do_fail()
        else:
            self.do_pass()

    def sendall_recv(self, jsondata, methods = 1):
        '''
        为发送的json数据添加methods和length报头，并等待接收方的回复
            POST: methods = 1
            REFUSE: methods = 9
        '''
        self.sock.setblocking(False)  # 将socket设置为非阻塞模式
        body = json.dumps(jsondata)
        header = [methods, body.__len__()]
        headPack = struct.pack(TcpSocket.headformat , *header)
        try:
            self.sock.sendall(headPack+body.encode())
        except:
            self.do_fail()
        else:
            ready_to_read, _, _ = select.select([self.sock], [], [], 3)
            if ready_to_read:
                try:
                    data = self.sock.recv(1024)
                    self.do_pass()
                except:
                    self.do_fail()
                else:
                    if not data: # 连接断开
                        self.do_fail()
            else: # 超时
                self.do_fail()

    @staticmethod
    def recv(conn):
        '''
        循环接收数据，直到收完报头中length长度的数据
        '''
        dataBuffer = bytes()
        while True:
            data = conn.recv(1024)
            # if data == b"":
            #     break
            if data:
                dataBuffer += data
                while True:
                    if len(dataBuffer) < TcpSocket.headerSize:
                        break
                    # 读取包头
                    headPack = struct.unpack(TcpSocket.headformat, dataBuffer[:TcpSocket.headerSize])
                    bodySize = headPack[1]
                    if len(dataBuffer) < TcpSocket.headerSize+bodySize :
                        break
                    body = dataBuffer[TcpSocket.headerSize:TcpSocket.headerSize+bodySize]
                    body = body.decode()
                    body = json.loads(body)
                    return headPack,body

    def set_keep_alive(self):
        """
        设置套接字保活机制
        30s后没反应开始探测连接，30s探测一次，一共探测10次，失败则断开
        """
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        if platform.system() == "Windows":
            self.sock.ioctl(socket.SIO_KEEPALIVE_VALS,(1,30*1000,30*1000))

        if platform.system() == "Linux":
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 30)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 30)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 10)

    def do_pass(self):
        if self.state == "failing":
            self.owner.sendRunDatatoGUI(f"Reconnected successfully with neighbor node {self.ID}.") 
        self.state = "passing"
        self.fail_count = 0

    def do_fail(self):
        if self.fail_count == 0:
            self.first_fail_time = datetime.datetime.now()
            self.state = "failing"
            self.thread = threading.Thread(target=self.reconnect)
            self.thread.start()
            self.owner.sendRunDatatoGUI(f"Connection to neighbor node {self.ID} failed, trying to reconnect.") 
        self.fail_count += 1
        if self.fail_count >= self.fail_limit and datetime.datetime.now()-self.first_fail_time >= datetime.timedelta(seconds=self.fail_limit*self.time_interval):
            self.state = "failed"
            # 关闭节点连接
            self.owner.deleteTaskNbrID(self.ID)

    def reconnect(self):
        while self.state == "failing":
            time.sleep(self.time_interval)
            try:
                self.sock.close()
            except:
                pass
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.set_keep_alive()
                self.sock.settimeout(5)
                self.sock.connect((self.ip, self.port))
            except:
                self.do_fail()
            else:
                self.do_pass()


class DaspCommon():
    '''
    Dasp公共变量及函数


    属性:
        COMMRANGE: 通信范围
        nodeID: 节点ID
        IP: 节点IP
        Port: 节点端口列表
        nbrID: 邻居ID
        addNbrIDFlag: 邻居ID添加完成标志
        wiredNbrID: 有线邻居ID
        wiredlessNbrID: 无线邻居ID
        location: 节点位置
        wiredless: 无线标志
        nodesIpDict: 节点IP字典
        nodesPortdict: 节点端口字典
        assignedPortDict: 节点为邻居分配的端口字典
        nbrSocket: 邻居通信套接字
        GuiInfo: UI界面IP及端口
        headformat: 自定义消息头格式，methods+length，2个无符号整形变量
        headerSize: 自定义消息头长度，8个字节
        systemFlag: 系统标志，用于判断系统是否初始化完成
        (包括读取拓扑文件的信息，以及所有类共有的信息)
        commServerThread: 通信服务器线程(邻居连接)
        taskServerThread: 任务服务器线程(任务接收)
        systemTaskThread: 系统任务线程
    '''
    # 类变量，直接通过DaspCommon.维护
    COMMRANGE = 10
    nodeID = ""
    IP = ""
    Port = []
    nbrID = []
    nbrDirection = []
    wiredNbrID = []
    wiredlessNbrID = []
    location = {"X": 0, "Y": 0, "Z": 0}
    wiredless = 1
    nbrSocket = {}
    addNbrIDFlag = {}
    mapSocket = None
    GuiInfo = ["localhost",50000,50001]    
    nodesIpDict = {}
    nodesPortdict = {}
    assignedPortDict = {}
    headformat = "!2I"
    headerSize = 8
    systemFlag = False

    commServerThread = []
    taskServerThread = []
    systemTaskThread = []

    def __init__(self):
        pass

    def sendtoGUIbase(self, info, key, DappName):
        """
        通过UDP的形式将信息发送至GUI
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            addr = (DaspCommon.GuiInfo[0], DaspCommon.GuiInfo[1])
            data = {
                "key": key,
                "id": DaspCommon.nodeID,
                "DappName":DappName,
                "time": datetime.datetime.now().strftime("%m-%d %H:%M:%S.%f")[:-3],
                "info": info
            }
            sock.sendto(json.dumps(data).encode('utf-8'), addr)
            sock.close()
        except Exception as e:
            print ("Failed to send" + info)

    def deleteNbrID(self, id):  
        """
        删除本节点和指定id邻居节点的所有连接(DaspCommon类变量)
        """
        
        if id in DaspCommon.nbrID:
            index = DaspCommon.nbrID.index(id)      
            del DaspCommon.nbrID[index]
            del DaspCommon.nbrDirection[index]
        if id in DaspCommon.wiredNbrID:
            index = DaspCommon.wiredNbrID.index(id)  
            del DaspCommon.wiredNbrID[index]
        if id in DaspCommon.wiredlessNbrID:
            index = DaspCommon.wiredlessNbrID.index(id) 
            del DaspCommon.wiredlessNbrID[index]
        if id in DaspCommon.nbrSocket:
            del DaspCommon.nbrSocket[id]
        if id in DaspCommon.addNbrIDFlag:
            del DaspCommon.addNbrIDFlag[id]

    def _async_raise(self, tid, exctype):
        '''
        主动抛出异常
        '''
        tid = ctypes.c_long(tid)
        if not inspect.isclass(exctype):
            exctype = type(exctype)
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
        if res == 0:
            raise ValueError("invalid thread id")
        elif res != 1:
            # """if it returns a number greater than one, you're in trouble,
            # and you should call it again with exc=NULL to revert the effect"""
            ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
            raise SystemError("PyThreadState_SetAsyncExc failed")

    def stop_thread(self, thread):
        '''
        强制停止某个线程
        '''
        self._async_raise(thread.ident, SystemExit)
