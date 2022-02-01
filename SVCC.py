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
import Traveltime


def DP(platoon):
    signal = {}
    c = 0
    first = [0] * 16
    while True:
        start, end = c * H, c * H + H
        # 识别本周期内考虑的车队
        nowpla = Supplymethods.considered(platoon, first, end, L)
        # print('Cycle: ', c)
        # 搜索最有信号配时方案
        signal[c] = signaltiming(nowpla, c)
        # 记录需要在下周期考虑的车辆开始索引
        first = Supplymethods.recordlast(first, nowpla, end)
        # print('First in next cycle: ', first)
        c += 1
        if c > T / H:
            break
        else:
            continue
    return signal


# 搜索最优信号配时
def signaltiming(state, c):
    green = [[], [], [], []]
    # Forward recursion
    j = 0
    stateset = np.arange(minG + clt, H + 1, tstep)
    stateset = np.insert(stateset, 0, 0)
    # print('stateset', stateset)
    funcJ = {}
    optJSX = {}
    while True:
        if j == 0:
            optfuncS = {}
            optSX = {}
            for sj in stateset:
                xj = sj - clt if sj - clt >= minG else 0
                valueX = {}
                signal = Supplymethods.firstgreen(c, xj, clt, H)
                # print('sj:', sj, 'xj:', xj, 'signal:', signal)
                valueX[xj] = Traveltime.averagetime(state, m2p, signal, L, H, toff, doff, c)
                # print('valueX', valueX)
                optfuncS[sj] = valueX[xj]
                optSX[sj] = xj
                # print('optfuncS', optfuncS)
            funcJ[j] = optfuncS
            optJSX[j] = optSX
            # print('optfuncS:', optfuncS)
            # print('optJSX:', optJSX)
            # print('funcJ', funcJ)
        else:
            optfuncS = {}
            optSX = {}
            for sj in stateset:
                Xsj = Supplymethods.feasibleX(minG, clt, tstep, sj)
                # print('sj:', sj, 'Xsj:', Xsj)
                valueX = {}
                for xj in Xsj:
                    sj_1 = sj if xj == 0 else sj - xj - clt
                    if sj_1 in stateset:
                        signal = Supplymethods.greenintervals(c, j, sj_1, xj, H, optJSX, clt)
                        avertime = Traveltime.averagetime(state, m2p, signal, L, H, toff, doff, c)
                        valueX[xj] = avertime
                    else:
                        continue
                optx = Supplymethods.optX(valueX)
                # print('optx', optx)
                optfuncS[sj] = valueX[optx]
                optSX[sj] = optx
            funcJ[j] = optfuncS
            optJSX[j] = optSX
            # print('optJSX:', optJSX)
            # print('funcJ', funcJ)
        # if j == 1:
        #     break
        if j > H / clt:
            break
        if j >= 1 and 0 <= (funcJ[j-1][H] - funcJ[j][H]) / funcJ[j-1][H] < 0.05:
            break
        j += 1
    # Backward recursion
    sbj = H
    while j >= 0:
        xbj = optJSX[j][sbj]
        duration = xbj + clt if xbj > 0 else 0
        p = j % 4
        if sbj > clt:
            green[p].append([sbj - duration, sbj - clt])
        else:
            green[p].append([sbj - duration, sbj])
        sbj -= duration
        j -= 1
    # print('c:', c, 'green: ', green)
    return green


# 生成给定信号配时下得所有车辆的轨迹，以及对应的平均行程时间
def trajectory(platoon, signal, T):
    P = []
    for i in range(16):
        phase = m2p[i]
        green = signal[phase]
        pla = platoon[i]
        moveP = []
        for n in range(len(pla)):
            veh = pla[n]
            if n == 0:
                veh.linit = None
                veh.lp = None
                print('第 0 辆：', veh.init)
                if veh.init[3] == 1:
                    p = veh.SH(green, L, T)
                else:
                    p = veh.H_SH(green, L, T)
                print('Tra：', p)
                moveP.append(p)
            else:
                veh.linit = pla[n - 1].init
                veh.lp = moveP[n - 1]
                print('第', n, '辆：', veh.init)
                if veh.init[3] == 1:
                    p = veh.SH(green, L, T)
                else:
                    p = veh.H_SH(green, L, T)
                print('Tra：', p)
                moveP.append(p)

        P.append(moveP)
    return P


# 生成所有车辆
def generation(init):
    platoon = {}
    for i in range(16):
        move = []
        state = init[i]
        for n in range(len(state)):
            veh = Vehicle(a1, a2, toff, doff, n, state[n])
            move.append(veh)
        platoon[i] = move
    return platoon


# Plot the signal timing and vehicle trajectories
def showtrajectory(platoon, P, signal, folder):
    for i in range(16):
        mp = P[i]
        pla = platoon[i]
        phase = m2p[i]
        green = signal[phase]
        filename = folder + '\\Movement' + str(i)
        Supplymethods.plotTra(pla, mp, L, green, T, clt, filename)


# 主函数
if __name__ == "__main__":
    # scenario = 0
    for scenario in range(2):
        if scenario == 0:
            file = 'data\\InitialStates.xls'
            folder = 'figure\\DLA'
        else:
            file = 'data\\TradInitialStates.xls'
            folder = 'figure\\Fixed'
        state = W_R_data.readinit(file)
        platoon = generation(state)
        green = DP(platoon)
        signal = Supplymethods.regulargreen(green, H, clt)
        print('signal:', signal)
        P = trajectory(platoon, signal, T)
        Supplymethods.savetraveltime(P, scenario)
        showtrajectory(platoon, P, signal, folder)

