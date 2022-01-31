# -*- coding: utf-8 -*-
"""
Created on Date 2022-01-25

@author: Rongjian Dai
"""
import numpy as np
from Vehicle import *


# 给定车辆初始状态和信号配时，计算平均行程时间
def averagetime(nowpla, m2p, signal, L, H, toff, doff, c):
    traveltime = []
    for i in range(16):
        phase = m2p[i]
        # print('Phase:', phase)
        pla = nowpla[i]
        green = signal[phase]
        arrival = []
        for n in range(len(pla)):
            init = pla[n].init
            fast = fastarrival(init, L, pla[n].a1)
            if n == 0:
                exparrive = canpass(fast, green, H, c)
                # print('Fast:', fast, 'Expectarrival: ', exparrive)
                arrival.append(exparrive)
                traveltime.append(exparrive - init[0])
            else:
                headway = (toff[0] + doff[0] / init[2]) if init[3] == 1 else (toff[1] + doff[1] / init[2])
                leadarrive = arrival[n - 1]
                mayarrive = leadarrive + headway if fast < leadarrive + headway else fast
                exparrive = canpass(mayarrive, green, H, c)
                # print('Fast:', fast, 'Expectarrival: ', exparrive)
                arrival.append(exparrive)
                traveltime.append(exparrive - init[0])

    return np.mean(traveltime)


# 不受前车影响时，判断是否绿灯能够通过，如不能给出期望通过时间
def canpass(fast, green, H, c):
    exparrive = 0
    can = False
    for i in range(len(green)):
        if green[i][0] <= fast <= green[i][1] and green[i][1] - green[i][0] > 0:
            exparrive = fast
            can = True
            break
        else:
            continue
    if can is False:
        for i in range(len(green)):
            if fast < green[i][0]:
                exparrive = green[i][0]
                break
            else:
                continue
    if exparrive == 0:
        exparrive = (c + 1) * H
    return exparrive


# The possible earliest arrival time
def fastarrival(init, L, a1):
    (t0, v0, vmax) = (init[0], init[1], init[2])
    # 肯定能够加速到最大速度，因此不作判断，直接使用两段轨迹计算
    t1 = (vmax - v0) / a1
    t2 = (L - (vmax ** 2 - v0 ** 2) / (2 * a1)) / vmax
    return t0 + t1 + t2



