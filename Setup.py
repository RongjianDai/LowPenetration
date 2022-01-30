# -*- coding: utf-8 -*-
"""
Created on Date

@author: Rongjian Dai
"""

import numpy as np
import LA


# Initialize the slope of Q
def qvariation(turning, basicQ, T, vt):
    """
    :param basicQ: 基础流量
    :param T: 仿真时长
    :param vt: 变化模式
    :return:
    """
    s = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
    for i in range(4):
        # 提取各转向基础流量
        (lt, sa, rt) = (turning[i][0], turning[i][1], turning[i][2])
        (lq, sq, rq) = (basicQ[i][lt], basicQ[i][sa], basicQ[i][rt])
        # 计算差值及变化率
        if vt[i] == 0:    # 不变化
            pass
        elif vt[i] == 1:    # left - straight
            k = (lq - sq) / T
            s[i][lt] = - k
            s[i][sa] = k
        elif vt[i] == 2:    # right - straight
            k = (rq - sq) / T
            s[i][rt] = - k
            s[i][sa] = k
        elif vt[i] == 3:    # left - right
            k = (lq - rq) / T
            s[i][lt] = - k
            s[i][rt] = k
    return s


def initialize(basicQ, slope, vmax, P, T):
    """
    初始化：生成车辆初始状态
    :param basicQ: 基础交通流[4][4]列表
    :param vmax: 最大速度1*3元组
    :param P: 渗透率
    :param T: 周期时长
    :return:
    """
    initVector = {}
    for i in range(4):
        for j in range(4):
            # 生成初始状态
            if j == i:
                pass
            else:
                move = str(i) + '_' + str(j)
                init = genveh(basicQ[i][j], vmax[i][j], P, slope[i][j], T)
                initVector[move] = init
    return initVector


def genveh(q, vmax, P, slope, T):
    init = []
    # 第一辆车
    t0 = np.random.uniform(0, 5)
    type = 1 if np.random.random() <= P else 0
    v0 = np.random.uniform(0.8*vmax, vmax)
    init.append([t0, v0, vmax, type])
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
            init.append([t_, v0, vmax, type])

    return init


# Real-time traffic volume for a turning movement
def qt(time, q0, slope):
    rq = q0 + slope * time
    return rq


# Real-time traffic volume matrix
def Qt(H, T, basicQ, slope):
    QT = {}
    c = 1
    QT[0] = basicQ
    while True:
        clock = c * H
        if clock > T:
            break
        else:
            q = []
            for i in range(4):
                qa = []
                for j in range(4):
                    qa.append(qt(clock, basicQ[i][j], slope[i][j]))
                q.append(qa)
            QT[c] = q
        c += 1
    return QT


# 计算专用直行车道通行能力
def sat_ET(P, toff, doff):
    tmean = toff[0] * P + toff[1] * (1 - P)
    dmean = doff[0] * P + doff[1] * (1 - P)
    sET = 3600 / (tmean + dmean / 15)
    return sET


# 计算各类型车道通行能力
def saturation(sET):
    sEL = sET / 1.125
    sER = sET / 1.25
    sLT = sET / (1 + (1.125 - 1) * 0.5)
    sRT = sET / (1 + (1.25 - 1) * 0.5)
    # print('EL:', sEL, 'ER:', sER, 'LT:', sLT, 'RT:', sRT)
    return sEL, sLT, sET, sRT, sER


# 计算供给向量表
def supply(S, schemes):
    V = []
    for i in range(6):
        v = []
        for j in range(6):
            (lt, sa, rt) = (0, 0, 0)
            if len(schemes[i][j]) == 4:
                for k in range(4):
                    if schemes[i][j][k] == 1:
                        lt += S[0]
                    elif schemes[i][j][k] == 2:
                        lt += 0.5 * S[1]
                        sa += 0.5 * S[1]
                    elif schemes[i][j][k] == 3:
                        sa += S[2]
                    elif schemes[i][j][k] == 4:
                        sa += 0.5 * S[3]
                        rt += 0.5 * S[3]
                    else:
                        rt += S[4]
                v.append([lt, sa, rt])
            else:
                v.append([])
        V.append(v)
    return V


# 车道功能优化
def lanemarking(QT, sET):
    """
    :param sET: 直行车道通行能力
    :param QT: 每个控制周期的流量矩阵
    :return:
    """
    marking = {}
    mulit = {}
    for (c, Q) in QT.items():
        # 优化LA
        (fun, u) = LA.optimization(Q, sET)
        marking[c] = fun
        mulit[c] = u
    return marking, mulit


