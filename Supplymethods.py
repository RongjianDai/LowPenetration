# -*- coding: utf-8 -*-
"""
Created on Date 2022-01-23

@author: Rongjian Dai
"""

import numpy as np
from Scenario import *


# 记录剩余车辆索引
def recordlast(first, nowpla, end):
    last = [None] * 16
    for i in range(16):
        pla = nowpla[i]
        if len(pla) == 0:
            last[i] = first[i]
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
        move = platoon[i]
        s, e = first[i], len(move)
        for veh in move:
            fast = veh.fastarrival(L)
            if fast > end:
                e = veh.index
                break
            else:
                continue
        nowpla[i] = move[s:e]

    return nowpla


# Feasible set of xj given state sj
def feasibleX(minG, clt, tstep, sj):
    if sj - clt < minG:
        X = np.array([0])
    else:
        X = np.arange(0, sj - clt + 1, tstep)
    return X


# Given sj_1, xj, get the green intervals for each phase
def greenintervals(c, j, sj_1, xj, H, clt):
    green = [[], [], [], []]
    start, end = c * H, c * H + H
    for i in range(4):
        if i < j:
            green[i] = [start, start + sj_1]
        elif i == j:
            green[i] = [start + sj_1, start + sj_1 + xj + clt]
        else:
            green[i] = [start + sj_1 + xj + clt, end]
    return green


# Find the optimal control variable
def optX(valueX, Xsj):
    optx = Xsj[0]
    for xj in Xsj:
        if valueX[xj] < valueX[optx]:
            optx = xj
        else:
            continue
    return optx

green = greenintervals(0, 2, 40, 10, H, clt)
print(green)