# -*- coding: utf-8 -*-
"""
Created on Date 2022-01-14

@author: Rongjian Dai
"""

import numpy as np


# 判断是否需要重新优化LA，返回每个周期的LA，存为字典
def reoptLA(QT, marking, supTab, schemeset, turning):
    lascheme = {}
    armpattern = {}
    optc = [0]
    c = 0
    lascheme[0] = marking[0]
    armpattern[0] = lanepattern(lascheme[0], turning)
    # print('armpattern[0]:', armpattern[0])
    while True:
        c += 1
        if c in QT.keys():
            lastc = optc[-1]
            optQ = QT[lastc]
            nowQ = QT[c]
            optpat = lanepattern(marking[lastc], turning)
            # print('cycle:', c)
            c1 = firstceritrion(optQ, nowQ, turning)
            c2 = secondceritrion(optpat, nowQ, supTab, schemeset, turning)
            if c1 is True:   # 需求模式相同，不需重新优化
                lascheme[c] = marking[lastc]
                continue
            elif c2 is True:   # 存在相似度更大的方案，需重新优化
                lascheme[c] = marking[c]
                optc.append(c)
            else:
                lascheme[c] = marking[lastc]
            # 给出对应arm pattern
            armpattern[c] = lanepattern(lascheme[c], turning)
        else:
            break

    return lascheme, armpattern, optc


# 准则1，demand vector变化情况
def firstceritrion(optQ, nowQ, turning):
    c1 = True    # 假设变化方向是共线的
    for i in range(4):
        (lt, sa, rt) = (turning[i][0], turning[i][1], turning[i][2])
        optdemv = np.array([optQ[i][lt], optQ[i][sa], optQ[i][rt]])
        nowdemv = np.array([nowQ[i][lt], nowQ[i][sa], nowQ[i][rt]])
        # 计算叉积 cross product
        crossp = np.cross(optdemv, nowdemv)
        if np.any(crossp):    # 只要有一个Arm不平行就可能需要重新优化，不需继续判断
            c1 = False
            break
        else:
            continue
    return c1


# 准则2，cosine similarity比较
def secondceritrion(optpat, nowQ, supTab, schemeset, turning):
    c2 = False
    for i in range(4):
        (lt, sa, rt) = (turning[i][0], turning[i][1], turning[i][2])
        nowdemv = np.array([nowQ[i][lt], nowQ[i][sa], nowQ[i][rt]])   # 当前demand vector
        nowindex = findindex(optpat[i], schemeset)      # LA模式索引
        # print('optpat[i]:', optpat[i], 'nowindex:', nowindex)
        nowsupv = np.array(supTab[nowindex[0]][nowindex[1]])      # 当前supply vector
        nowsim = similarity(nowdemv, nowsupv)        # 当前相似度
        # 找出备选方案集合
        neigind = []
        if nowindex[0] + 1 < 6:
            if nowindex[1] < 5 - nowindex[0]:
                neigind.append([nowindex[0] + 1, nowindex[1]])
            if nowindex[1] - 1 >= 0:
                neigind.append([nowindex[0] + 1, nowindex[1] - 1])
        if nowindex[0] - 1 >= 0:
            neigind.append([nowindex[0] - 1, nowindex[1]])
            if nowindex[1] + 1 < 7 - nowindex[0]:
                neigind.append([nowindex[0] - 1, nowindex[1] + 1])
        if nowindex[1] + 1 < 6 - nowindex[0]:
            neigind.append([nowindex[0], nowindex[1] + 1])
        if nowindex[1] - 1 >= 0:
            neigind.append([nowindex[0], nowindex[1] - 1])
        # 准则2
        for index in neigind:
            potsupv = np.array(supTab[index[0]][index[1]])      # potential supply vector
            potsim = similarity(nowdemv, potsupv)       # Corresponding cosine similarity
            # print('Arm', i, 'nowdem:', nowdemv, 'potsupv', potsupv, 'potsim', potsim, 'nowsim', nowsim)
            if potsim > nowsim:
                c2 = True
                break
            else:
                continue
        if c2 is True:     # 只要有一个Arm相似度符合在优化准则，不需继续判断
            break
        else:
            continue
    return c2


# 计算 cosine similarity
def similarity(v1, v2):
    sim = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    return sim


# 判断lane-group模式
def lanepattern(marking, turning):
    schemes = []    # [4][4]list，4 arms, 4 lanes
    for i in range(4):
        (lt, sa, rt) = (turning[i][0], turning[i][1], turning[i][2])
        scheme = []
        for k in range(4):
            # Arm i 上lane k 为: EL
            if marking[i][lt][k] == 1 and marking[i][sa][k] == 0:
                scheme.append(1)
            # Arm i 上lane k 为: LT
            if marking[i][lt][k] == 1 and marking[i][sa][k] == 1:
                scheme.append(2)
            # Arm i 上lane k 为: ET
            if marking[i][sa][k] == 1 and marking[i][lt][k] == 0 and marking[i][rt][k] == 0:
                scheme.append(3)
            # Arm i 上lane k 为: TR
            if marking[i][sa][k] == 1 and marking[i][rt][k] == 1:
                scheme.append(4)
            # Arm i 上lane k 为: ER
            # print('marking[i][sa][k]:', marking[i][sa][k], 'marking[i][rt][k]:', marking[i][rt][k])
            if marking[i][sa][k] == 0 and marking[i][rt][k] == 1:
                scheme.append(5)
        schemes.append(scheme)
    return schemes


# 确定某一个Arm 的LA scheme对应的supply vector
def findindex(optpat, schemeset):
    for i in range(6):
        for j in range(6 - i):
            if optpat[0] == schemeset[i][j][0] and optpat[1] == schemeset[i][j][1] and optpat[2] == schemeset[i][j][2] and optpat[3] == schemeset[i][j][3]:
                index = [i, j]
                return index
            else:
                continue
