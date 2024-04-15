import socket
import sys
import threading
import time
sys.path.insert(1,".")  # 把上一级目录加入搜索路径
from DASP.module import DaspCommon, TaskServer, CommServer, MapSocket

class Server(object):
    def __init__(self, ID, GuiInfo, nodesIpDict, nodesPortdict, COMMRANGE, wiredNbrID, IP,Port,location,wiredless):
        DaspCommon.nodeID = ID
        DaspCommon.GuiInfo = GuiInfo
        DaspCommon.nodesIpDict = nodesIpDict
        DaspCommon.nodesPortdict = nodesPortdict
        DaspCommon.COMMRANGE = COMMRANGE
        DaspCommon.IP = socket.gethostbyname(IP)
        DaspCommon.Port = Port
        DaspCommon.nbrID = wiredNbrID
        DaspCommon.nbrDirection = [ i+1 for i in range(len(wiredNbrID))]
        DaspCommon.wiredNbrID = wiredNbrID
        DaspCommon.location = location
        DaspCommon.wiredless = wiredless

        DaspCommon.wiredlessNbrID = []
        DaspCommon.commServerThread = []
        DaspCommon.taskServerThread = []
        DaspCommon.systemTaskThread = []
        DaspCommon.assignedPortDict = {}

        DaspCommon.mapSocket = MapSocket(DaspCommon.nodeID, DaspCommon.GuiInfo[0], DaspCommon.GuiInfo[2])

    def run(self):
        #创建接口服务器
        # for _,port in enumerate(DaspCommon.Port[1:]):
        #     commserver = CommServer(DaspCommon.IP, port)
        #     t = threading.Thread(target=commserver.run,args=())
        #     t.setDaemon(True)
        #     self.commServerThread.append(t)
        taskserver = TaskServer(DaspCommon.IP,DaspCommon.Port)
        DaspCommon.taskServerThread = threading.Thread(target=taskserver.run,args=())
        DaspCommon.taskServerThread.setDaemon(True)
        DaspCommon.systemTaskThread = threading.Thread(target=taskserver.systemtask,args=())
        DaspCommon.systemTaskThread.setDaemon(True)
        # for _,thread in enumerate(self.commServerThread):
        #     thread.start()
        DaspCommon.taskServerThread.start()

    def runSystemTask(self):
        DaspCommon.systemTaskThread.start()
        while True:
            time.sleep(1)

        

