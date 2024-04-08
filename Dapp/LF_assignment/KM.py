#K-M算法计算分配编号
import numpy as np

class KM:
    def __init__(self, origin_formation, target_formation):
        self.origin_formation = np.transpose(origin_formation)
        self.target_formation = np.transpose(target_formation)
        self.uav_num = self.origin_formation.shape[1] 

        # 将origin_formation的每列减去第一列，并去除第一列得到一个新的矩阵
        self.origin_formation = self.origin_formation - self.origin_formation[:, 0].reshape(3, 1)
        self.origin_formation = self.origin_formation[:, 1:]
        self.target_formation = self.target_formation - self.target_formation[:, 0].reshape(3, 1)
        self.target_formation = self.target_formation[:, 1:]

        # 计算两个矩阵的距离矩阵
        self.adj_matrix = self.build_graph()

        self.label_left = np.max(self.adj_matrix, axis=1)  # init label for the left set
        self.label_right = np.array([0.] * (self.uav_num - 1))  # init label for the right set
        self.match_right = np.array([-1] * (self.uav_num - 1))
        self.visit_left = np.array([0] * (self.uav_num - 1))
        self.visit_right = np.array([0] * (self.uav_num - 1))
        self.slack_right = np.array([100.0] * (self.uav_num - 1))
        self.changed_id = self.KMcore()
        # Get a new formation pattern of UAVs based on KM.
        self.new_formation = self.get_new_formation(self.changed_id, self.target_formation)
        print(self.new_formation)
        self.communication_topology = self.get_communication_topology(self.new_formation)
        print(self.communication_topology)

    def build_graph(self):
        distance = [[0 for i in range(self.uav_num - 1)] for j in range(self.uav_num - 1)]
        for i in range(self.uav_num - 1):
            for j in range(self.uav_num - 1):
                distance[i][j] = np.linalg.norm(self.origin_formation[:, i] - self.target_formation[:, j])
                distance[i][j] = 50 - distance[i][j]
        return distance


    def find_path(self, i):
        self.visit_left[i] = True
        for j, match_weight in enumerate(self.adj_matrix[i], start=0):
            if self.visit_right[j]:
                continue
            gap = self.label_left[i] + self.label_right[j] - match_weight
            if gap == 0:
                self.visit_right[j] = True
                if self.match_right[j] == -1 or self.find_path(self.match_right[j]):
                    self.match_right[j] = i
                    return True
            else:
                self.slack_right[j] = min(gap, self.slack_right[j])
        return False

    # Main body of KM algorithm.
    def KMcore(self):
        for i in range(self.uav_num - 1):
            self.slack_right = np.array([100.] * (self.uav_num - 1))
            while True:
                self.visit_left = np.array([0] * (self.uav_num - 1))
                self.visit_right = np.array([0] * (self.uav_num - 1))
                if self.find_path(i):
                    break
                d = np.inf
                for j, slack in enumerate(self.slack_right):
                    if not self.visit_right[j]:
                        d = min(d, slack)
                for k in range(self.uav_num - 1):
                    if self.visit_left[k]:
                        self.label_left[k] -= d
                    if self.visit_right[k]:
                        self.label_right[k] += d
                    else:
                        self.slack_right[k] -= d
        return self.match_right

    def get_new_formation(self, changed_id, change_formation):
        new_formation = np.zeros((3, self.uav_num - 1))
        position = np.zeros((3, self.uav_num - 1))
        changed_id = [i + 1 for i in changed_id]
        for i in range(0, self.uav_num - 1):
            position[:, i] = change_formation[:, i]

        for i in range(0, self.uav_num - 1):
            for j in range(0, self.uav_num - 1):
                if (j + 1) == changed_id[i]:
                    new_formation[:, i] = position[:, j]
        return new_formation

    def get_communication_topology(self, rel_posi):
        c_num = int((self.uav_num) / 2)
        min_num_index_list = [0] * c_num

        comm = [[] for i in range(self.uav_num)]
        communication = np.ones((self.uav_num, self.uav_num)) * 0
        nodes_next = []
        node_flag = [self.uav_num - 1]
        node_mid_flag = []

        rel_d = [0] * (self.uav_num - 1)

        for i in range(0, self.uav_num - 1):
            rel_d[i] = pow(rel_posi[0][i], 2) + pow(rel_posi[1][i], 2) + pow(rel_posi[2][i], 2)

        c = np.copy(rel_d)
        c.sort()
        count = 0

        for j in range(0, c_num):
            for i in range(0, self.uav_num - 1):
                if rel_d[i] == c[j]:
                    if not i in node_mid_flag:
                        min_num_index_list[count] = i
                        node_mid_flag.append(i)
                        count = count + 1
                        if count == c_num:
                            break
            if count == c_num:
                break

        for j in range(0, c_num):
            nodes_next.append(min_num_index_list[j])

            comm[self.uav_num - 1].append(min_num_index_list[j])

        size_ = len(node_flag)

        while (nodes_next != []) and (size_ < (self.uav_num - 1)):

            next_node = nodes_next[0]
            nodes_next = nodes_next[1:]
            min_num_index_list = [0] * c_num
            node_mid_flag = []
            rel_d = [0] * (self.uav_num - 1)
            for i in range(0, self.uav_num - 1):

                if i == next_node or i in node_flag:

                    rel_d[i] = 2000
                else:

                    rel_d[i] = pow((rel_posi[0][i] - rel_posi[0][next_node]), 2) + pow(
                        (rel_posi[1][i] - rel_posi[1][next_node]), 2) + pow((rel_posi[2][i] - rel_posi[2][next_node]),
                                                                            2)
            c = np.copy(rel_d)
            c.sort()
            count = 0

            for j in range(0, c_num):
                for i in range(0, self.uav_num - 1):
                    if rel_d[i] == c[j]:
                        if not i in node_mid_flag:
                            min_num_index_list[count] = i
                            node_mid_flag.append(i)
                            count = count + 1
                            if count == c_num:
                                break
                if count == c_num:
                    break
            node_flag.append(next_node)

            size_ = len(node_flag)

            for j in range(0, c_num):

                if min_num_index_list[j] in node_flag:

                    nodes_next = nodes_next

                else:
                    if min_num_index_list[j] in nodes_next:
                        nodes_next = nodes_next
                    else:
                        nodes_next.append(min_num_index_list[j])

                    comm[next_node].append(min_num_index_list[j])

        for i in range(0, self.uav_num):
            for j in range(0, self.uav_num - 1):
                if i == 0:
                    if j in comm[self.uav_num - 1]:
                        communication[j + 1][i] = 1
                    else:
                        communication[j + 1][i] = 0
                else:
                    if j in comm[i - 1] and i < (j+1):
                        communication[j + 1][i] = 1
                    else:
                        communication[j + 1][i] = 0
            
        for i in range(1, self.uav_num):  # 防止某个无人机掉队
            if sum(communication[i]) == 0:
                communication[i][0] = 1
        return communication

    def get_formation(self):
        return self.new_formation

if __name__ == "__main__":

    formation = {
                "origin":[[0,3,-2],[0,0,-2],[0,6,-2],[-3,0,-2],[-3,3,-2],[-3,6,-2],[-6,0,-2],[-6,3,-2],[-6,6,-2]],
                "diamond":[[0,0,0],[-3,-3,0],[-3,3,0],[-6,-6,0],[-6,6,0],[-9,-9,0],[-9,-3,0],[-9,3,0],[-9,9,0]]
            }
    # origin_formation = formation["origin"]
    # target_formation = formation["diamond"]
    origin_formation = [[0,0,0],[3,0,0],[0,3,0],[3,3,0]]
    target_formation = [[0,0,0],[0,1,0],[3,3,0],[3,0,0]]
    new_formation = KM(origin_formation, target_formation).get_formation()
    new_formation = np.transpose(new_formation)
    print(new_formation)