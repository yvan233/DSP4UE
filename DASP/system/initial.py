import sys
import codecs
import json
import os
import socket
import system
if __name__ == '__main__':
    IP = []
    Port = []
    ID = []
    wiredNbrID = []
    Data = []
    nodesIpDict = {}
    nodesPortdict = {}
    # nbrIpDict = {}
    # nbrPortdict = {}
    localIP = socket.gethostbyname(socket.gethostname())
    path = os.getcwd() + "/Dapp/Base/topology.json"
    text = codecs.open(path, 'r', 'utf-8').read()
    js = json.loads(text)
    COMMRANGE = js["CommRange"]
    for ele in js["Nodes"]:
        if "ID" in ele:
            ID.append(ele["ID"])
            IP.append(localIP)
            Port.append(ele["Port"])
            wiredNbrID.append(ele["WiredNbrID"])
            nodesIpDict[ele["ID"]] = localIP
            nodesPortdict[ele["ID"]] = ele["Port"]

    # order = 0
    order = int(sys.argv[1])
    selfID = ID[order]
    selfWiredNbrID = wiredNbrID[order]
    selfIP = IP[order]
    selfPort = Port[order]
    # nbrIpDict = {ele for ele in nodesIpDict.items() if ele[0] in selfWiredNbrID}
    # nbrPortdict = {ele for ele in nodesPortdict.items() if ele[0] in selfWiredNbrID}
    print(f"selfID: {selfID}, selfWiredNbrID: {selfWiredNbrID}, selfIP: {selfIP}, selfPort: {selfPort}")

    GuiInfo = [localIP, 50000]
    server = system.Server(selfID, GuiInfo, nodesIpDict, nodesPortdict, COMMRANGE, selfWiredNbrID, selfIP, selfPort)
    server.run()
    server.runSystemTask()