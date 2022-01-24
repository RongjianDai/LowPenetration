# -*- coding: utf-8 -*-
"""
Created on Date 2022-01-15

@author: Rongjian Dai
"""

import numpy as np
from Scenario import *
import W_R_data
from Vehicle import *
import Supplymethods


def DP(H, T, L, minG, clt, tstep, m2p, p2m, platoon):
    P = {}
    signal = {}
    c = 0
    first = [0] * 16
    while True:
        start, end = c * H, c * H + H
        # 识别本周期内考虑的车队
        nowpla = Supplymethods.considered(platoon, first, end, L)
        print('Cycle: ', c)
        # 搜索最有信号配时方案

        # 记录本周期内的轨迹和信号
        # 记录需要在下周期考虑的车辆开始索引
        first = Supplymethods.recordlast(first, nowpla, end)
        print('First in next cycle: ', first)
        c += 1
        if c > T / H:
            break
        else:
            continue
    return P


# 搜索最优信号配时
def signaltiming(nowpla, minG, clt, tstep, H, c):
    green = []
    # Forward recursion
    j = 0
    stateset = np.arange(clt, H + 1, tstep)
    funcJ = {}
    optJX = {}
    lastJ = 0
    while True:
        if j == 0:
            optfuncS = {}
            for sj in stateset:
                Xsj = Supplymethods.feasibleX(minG, clt, tstep, sj)
                valueX = {}
                green = Supplymethods.greenintervals(c, j, 0, xj, H, clt)
                valueX[xj] = trajectory(L, H, m2p, nowpla, green)
                optx = Supplymethods.optX(valueX, Xsj)
                optfuncS[sj] = valueX[optx]
            funcJ[j] = optfuncS
        else:
            optfuncS = {}
            for sj in stateset:
                Xsj = Supplymethods.feasibleX(minG, clt, tstep, sj)
                valueX = {}
                for xj in Xsj:
                    sj_1 = sj if xj == 0 else sj - xj - clt
                    green = Supplymethods.greenintervals(c, j, sj_1, xj, H, clt)
                    avertime = trajectory(L, H, m2p, nowpla, green)
                    valueX[xj] = avertime + optfuncS[j - 1][sj_1]
                optx = Supplymethods.optX(valueX, Xsj)
                optfuncS[sj] = valueX[optx]
            funcJ[j] = optfuncS
        if j >= 4:
            break
        # if j > H / clt:
        #     break
        if (funcJ[j-1][H] - funcJ[j][H]) / funcJ[j-1][H] < 0.05:
            break
        j += 1
    # Backward recursion

    return green


# 生成给定信号配时下得所有车辆的轨迹，以及对应的平均行程时间
def trajectory(L, H, m2p, nowpla, signal):
    traveltime = 0
    for i in range(16):
        p = m2p[i]
        green = signal[p]

    return traveltime


# 生成所有车辆
def generation(init, a1, a2, toff, doff):
    platoon = {}
    for i in range(16):
        move = []
        state = init[i]
        for n in range(len(state)):
            veh = Vehicle(a1, a2, toff, doff, n, state[n])
            move.append(veh)
        platoon[i] = move
    return platoon


# 主函数
if __name__ == "__main__":
    file = 'data\\InitialStates.xls'
    state = W_R_data.readinit(file)
    platoon = generation(state, a1, a2, toff, doff)
    DP(H, T, platoon, minG, clt, tstep, L)