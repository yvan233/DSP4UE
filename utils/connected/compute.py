import numpy as np
import scipy.linalg as linalg

def algebraic_connectivity(adj_matrix):
    # 计算度矩阵 D
    degree_matrix = np.diag(np.sum(adj_matrix, axis=1))
    
    # 计算拉普拉斯矩阵 L = D - A
    laplacian_matrix = degree_matrix - adj_matrix
    
    # 计算拉普拉斯矩阵的特征值
    eigvals = linalg.eigvals(laplacian_matrix)
    
    # 排序特征值并返回第二小的特征值
    eigvals = np.sort(np.real(eigvals))  # 只取实部，因为特征值应该是实数
    return eigvals[1]  # 第二小的特征值

# 示例：一个4个节点的图的邻接矩阵
adj_matrix = np.array([
    [0, 1, 1, 0,0,0],
    [1, 0, 1, 1,1,0],
    [1, 1, 0, 0,1,1],
    [0, 1, 0, 0,1,0],
    [0, 1, 1, 1,0,1],
    [0, 0, 1, 0,1,0]
])

# 计算代数连通度
algebraic_conn = algebraic_connectivity(adj_matrix)
print("代数连通度:", algebraic_conn)
