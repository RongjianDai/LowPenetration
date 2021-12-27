# -*- coding: utf-8 -*-
"""
Created on Date

@author: Rongjian Dai
"""

import numpy as np

# Simulation parameters
T = 3600
P = 0.2
L = 300
clear = 3
mingreen = 10
H = 90
(a1, a2) = (2, -8)
speed = (13.5, 15, 12)
toff = (1, 2)
doff = (5.5, 6.5)
# 基础交通需求
basicQ = [[0, 200, 400, 100],
          [150, 0, 150, 200],
          [380, 150, 0, 180],
          [100, 200, 100, 0]]
# 交通流方向索引
move = ['0_1', '0_2', '0_3', '1_0', '1_2', '1_3', '2_0', '2_1', '2_3', '3_0', '3_1', '3_2']
# 限速
vmax = [[0, 12, 15, 13.5], [13.5, 0, 12, 15], [15, 13.5, 0, 12], [12, 15, 13.5, 0]]
# 交通流量变化率
slope = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
# slope = [[0, 0, -0.0833, 0.0833], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]


def initialize(basicQ, vmax, P, slope, T):
    initVector = {}
    for i in range(4):
        for j in range(4):
            # 生成初始状态
            if j == i:
                pass
            else:
                move = str(i) + '_' + str(j)
                q = basicQ[i][j]
                init = genveh(q, vmax[i][j], P, slope[i][j], T)
                initVector[move] = init

    return initVector


def genveh(q, vmax, P, slope, T):
    init = []
    # 第一辆车
    t0 = np.random.uniform(0, 5)
    type = 1 if np.random.random() <= P else 0
    v0 = np.random.uniform(0.8*vmax, vmax)
    init.append([t0, v0, type])
    # 所有车
    clock = t0
    averh = 3600 / q
    while True:
        headway = np.random.exponential(averh, 1)[0]
        clock += headway
        averh = 3600 / qt(clock, q, slope)
        if clock >= T:
            break
        else:
            t_ = clock
            v0 = np.random.uniform(0.8 * vmax, vmax)
            type = 1 if np.random.random() <= P else 0
            init.append([t_, v0, type])

    return init


def qt(time, q0, slope):
    """
    :param time: 时刻
    :param q0: 初始流量
    :param slope: 斜率
    :return: 实时流量
    """
    q = q0 + slope * time
    return q


# 给出车辆初始状态信息
init = initialize(basicQ, vmax, P, slope, T)

# for m in move:
#     print(m, ':')
#     print(init[m])