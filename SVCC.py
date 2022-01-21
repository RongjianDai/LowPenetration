# -*- coding: utf-8 -*-
"""
Created on Date 2022-01-15

@author: Rongjian Dai
"""

import numpy as np
from Scenario import *
import W_R_data
from Vehicle import *


def DP(H, T, platoon, minG, clt, tstep, L):
    P = {}
    signal = []
    c = 0
    first = [0] * 16
    while True:
        start, end = c * H, c * H + H
        # 识别本周期内考虑的车队
        nowpla = considered(platoon, first, end, L)
        print('This cycle: ')
        # 搜索最有信号配时方案

        # 记录本周期内的轨迹和信号
        # 记录需要在下周期考虑的车辆开始索引
        first = recordlast(nowpla, end)
        print('First in next cycle: ', first)
        c += 1
        if c > T / H:
            break
        else:
            continue
    return P


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


# 记录剩余车辆索引
def recordlast(nowpla, end):
    last = [None] * 16
    for i in range(16):
        pla = nowpla[i]
        if len(pla) == 0:
            pass
        else:
            for veh in pla:
                if veh.fastarrival(300) > end:
                # if veh.p[-1][1] > end:  # 本周期内无法到达
                    last[i] = veh.index
                    break
                else:
                    continue
            # 若都能在本周期内到达，则记录最后一个车辆的index
            last[i] = (pla[-1].index + 1) if last[i] is None else last[i]
    return last


# 识别每个周期内考虑的车队
def considered(platoon, first, end, L):
    nowpla = {}
    for i in range(16):
        if first[i] is None:
            nowpla[i] = []
        else:
            move = platoon[i]
            s, e = first[i], len(move) - 1
            for veh in move:
                fast = veh.fastarrival(L)
                if fast > end:
                    e = veh.index
                    break
                else:
                    continue
            nowpla[i] = move[s:e]

    return nowpla


# 生成给定信号配时下得所有车辆的轨迹，以及对应的平均行程时间
def trajectory(platoon, signal, L, H):
    traveltime = 0
    for i in range(16):
        if 0 <= i < 4:
            phase = 0
        elif 4 <= i < 8:
            phase = 2
        elif 8 <= i < 12:
            phase = 1
        else:
            phase = 3

    return traveltime


# 主函数
if __name__ == "__main__":
    file = 'data\\InitialStates.xls'
    state = W_R_data.readinit(file)
    platoon = generation(state, a1, a2, toff, doff)
    DP(H, T, platoon, minG, clt, tstep, L)