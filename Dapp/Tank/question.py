# import需求模块
import time
from DASP.module import Task
from Connection.API_Utils import *
# 用户自定义函数区
"""
用户编码区域
"""
# 定义算法主函数taskFunction(self,id,nbrDirection,datalist)
# 四个形参分别为节点类，节点ID，节点邻居对应的方向以及初始化时键入的数据
def taskFunction(self:Task, id, nbrDirection, datalist):
    name = f'Tank_Red_{id}'
    self.sendDatatoGUI(name)
    time.sleep(3)
    go_forward(name, name, int(id))
    self.sendDatatoGUI(f"go forward {id}")
    return 0


