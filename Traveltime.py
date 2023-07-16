# -*- coding: utf-8 -*-
"""
Created on Date 2022-01-25

@author: Rongjian Dai
"""
import numpy as np
from Vehicle import *


# 给定车辆初始状态和信号配时，计算平均行程时间
def averagetime(nowpla, m2p, signal, L, H, toff, doff, c):
    delay = []
    for i in range(16):
        phase = m2p[i]
        # print('Phase:', phase)
        pla = nowpla[i]
        green = signal[phase]    # 一个相位的绿灯
        R = []
        Ae, T0 = [], []
        for n in range(len(pla)):
            init = pla[n].init
            fast = fastarrival(init, L, pla[n].a1)
            Ae.append(fast)
            T0.append(init[0])

        for n in range(len(pla)):
            if n == 0:
                if init[2] == 12:
                    R0 = Ae[0]
                else:
                    R0 = canpass(Ae[0], green, H, c)
                R.append(R0)
                delay.append(R0 - Ae[0])
            else:
                Rn_1 = R[n - 1]
                headway = (toff[0] + doff[0] / init[2]) if init[3] == 1 else (toff[1] + doff[1] / init[2])
                Rn = Ae[n] if Ae[n] >= Rn_1 + headway else Rn_1 + headway
                R.append(Rn)
                delay.append(Rn - Ae[n])

    return np.mean(delay)


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


def averdelay(nowpla, m2p, signal, L, H, toff, doff, c):
    delay = []
    end = (c + 1) * H
    for i in range(16):
        phase = m2p[i]
        # print('Phase:', phase)
        pla = nowpla[i]
        green = signal[phase]  # 一个相位的绿灯
        Ae, T0 = [], []
        numvehs = len(pla)
        for n in range(numvehs):
            init = pla[n].init
            fast = fastarrival(init, L, pla[n].a1)
            Ae.append(fast)
            T0.append(init[0])
        if numvehs > 0:
            R = []
            numgreen = len(green)
            if numgreen == 0:
                R0 = end
                R.append(R0)
                for n in range(1, numvehs):
                    Rn_1 = R[n - 1]
                    headway = (toff[0] + doff[0] / init[2]) if init[3] == 1 else (toff[1] + doff[1] / init[2])
                    Rn = Ae[n] if Ae[n] >= Rn_1 + headway else Rn_1 + headway
                    R.append(Rn)
            elif numgreen == 1:
                S, E = green[0][0], green[0][1]
                if E > S:
                    R0 = max(Ae[0], S)
                    R.append(R0)
                    for n in range(1, numvehs):
                        Rn_1 = R[n-1]
                        headway = (toff[0] + doff[0] / init[2]) if init[3] == 1 else (toff[1] + doff[1] / init[2])
                        Rn = Ae[n] if Ae[n] >= Rn_1 + headway else Rn_1 + headway
                        Rn = Rn if Rn <= E else end
                        R.append(Rn)
                else:
                    R0 = end
                    R.append(R0)
                    for n in range(1, numvehs):
                        Rn_1 = R[n-1]
                        headway = (toff[0] + doff[0] / init[2]) if init[3] == 1 else (toff[1] + doff[1] / init[2])
                        Rn = Ae[n] if Ae[n] >= Rn_1 + headway else Rn_1 + headway
                        R.append(Rn)
            elif numgreen > 1:
                R0 = end
                for j in range(numgreen):
                    if Ae[0] < green[j][0]:
                        R0 = green[j][0]
                        break
                    elif green[j][0] <= Ae[0] < green[j][1]:
                        R0 = Ae[0]
                        break
                    else:
                        continue
                R.append(R0)
                for n in range(1, numvehs):
                    Rn_1 = R[n - 1]
                    headway = (toff[0] + doff[0] / init[2]) if init[3] == 1 else (toff[1] + doff[1] / init[2])
                    Rn = Ae[n] if Ae[n] >= Rn_1 + headway else Rn_1 + headway
                    if Rn > green[numgreen-1][1]:
                        Rn = end
                    j = 0
                    while j < numgreen - 1:
                        if green[j][1] < Rn <= green[j+1][0]:
                            Rn = green[j+1][0]
                            break
                        j += 1

                    R.append(Rn)

            for n in range(numvehs):
                delay.append(R[n] - Ae[n])

        else:
            delay.append(0)

    return np.mean(delay)

