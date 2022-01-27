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
        X = np.arange(minG, sj - clt + 1, tstep)
        X = np.insert(X, 0, 0)
    return X


# Green of phase 0
def firstgreen(c, x0, clt, H):
    start, end = c * H, c * H + H
    if x0 != 0:
        green = [[[start, start + x0]],
                 [[start + x0 + clt, end]],
                 [[start + x0 + clt, end]],
                 [[start + x0 + clt, end]]]
    else:
        green = [[[start, start]],
                 [[start, end]],
                 [[start, end]],
                 [[start, end]]]
    return green


# Given sj_1, xj, get the green intervals for each phase
def greenintervals(c, j, sj_1, xj, H, optJSX, clt):
    green = [[], [], [], []]
    start, end = c * H, c * H + H
    p = j % 4
    # print('j:', j, 'p:', p)
    J = j - 1
    sback = sj_1
    while J >= 0:
        ep = J % 4
        optSX_1 = optJSX[J]
        duration = optSX_1[sback] + clt if optSX_1[sback] != 0 else 0
        if duration != 0:
            green[ep].append([start + sback - duration, start + sback - clt])
        else:
            green[ep].append([start + sback, start + sback])
        # print('Phase:', p, 'G: ', [start + sback - duration, start + sback])
        sback -= duration
        J -= 1

    for i in range(4):
        if i == p:
            if xj == 0:
                green[i].append([start + sj_1,  start + sj_1])
            else:
                green[i].append([start + sj_1, start + sj_1 + xj])
        if i > p:
            if xj == 0:
                green[i].append([start + sj_1, end])
            else:
                green[i].append([start + sj_1 + xj + clt, end])

    G = [[], [], [], []]
    for i in range(4):
        gp = green[i]
        for k in range(len(gp)):
            g = gp[k]
            if g[0] == g[1]:
                pass
            else:
                G[i].append(g)

    # print('j:', j, 'sj_1:', sj_1, 'xj:', xj, 'Green:', G)
    return G


# Find the optimal control variable
def optX(valueX):
    optx = 0
    for xj in valueX.keys():
        if valueX[xj] < valueX[optx]:
            optx = xj
        else:
            continue
    return optx


# Get the optimal signal timing
def retrivegreen(Xb):
    green = [[], [], [], []]

    return green