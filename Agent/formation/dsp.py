# Launch the system and run the distributed algorithm
import time 
import sys
sys.path.insert(1,".")
from DASP.module import Node,Moniter
from DASP.control import ControlMixin

nodeNum = 6  # Number of nodes
startNode = "1" # Starting node IDm
nodelist = [] # List of node processes
controlMixin = ControlMixin("Pc") # Collection of control functions

# Launch the monitoring script
moniter = Moniter()
moniter.run()

# Launch node processes
for i in range(nodeNum):
    node = Node(i)
    nodelist.append(node)

time.sleep(2)
DappName = "LF_avoid"
print("start task: "+DappName)
controlMixin.startTask(DappName,startNode)

moniter.wait()