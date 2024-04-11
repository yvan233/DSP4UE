from DASP.module import DaspCommon, Task
import random
import time
Rate = 1


def taskFunction(self:Task,id,nbrDirection,datalist):
    location = DaspCommon.location
    self.sendDatatoGUI(location)
    while True:
        last_time = time.time()
        location["X"] += random.randint(-1,1)
        location["Y"] += random.randint(-1,1)
        location["Z"] += random.randint(-1,1)
        
        self.updateTopology(location)
        # self.sendDatatoGUI(self.parentID)
        period = time.time() - last_time
        # 控制频率
        if period < 1/Rate:
            time.sleep(1/Rate - period) 
        

    return 0  