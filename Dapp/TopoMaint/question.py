from DASP.module import DaspCommon, Task
import random
import time
Rate = 1

def taskFunction(self:Task,id,nbrDirection,datalist):
    location = DaspCommon.location
    self.sendDatatoGUI(location)
    sptIter = 0
    while True:
        last_time = time.time()
        location["X"] += random.randint(-1,1)
        location["Y"] += random.randint(-1,1)
        location["Z"] += random.randint(-1,1)
        # 更新拓扑
        topology, nbrDistance = self.updateTopology(location)
        self.sendDatatoGUI(f"topology:{topology},nbrDistance:{nbrDistance}")
        
        sptIter += 1
        if sptIter == 5:
            # self.syncNode()
            tree = self.updateSpanningTree()
            self.sendDatatoGUI(tree)
            sptIter = 0
        period = time.time() - last_time
        # 控制频率
        if period < 1/Rate:
            time.sleep(1/Rate - period) 
        

    return tree