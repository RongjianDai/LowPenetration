# -*- coding: utf-8 -*-
"""
Created on Date 2022-01-23

@author: Rongjian Dai
"""

import numpy as np
from Scenario import *
import matplotlib.pyplot as plt
import xlsxwriter


# 记录剩余车辆索引
def recordlast(first, nowpla, end, L):
    last = [None] * 16
    for i in range(16):
        pla = nowpla[i]
        if len(pla) == 0:
            last[i] = first[i]
        else:
            for veh in pla:
                if veh.fastarrival(L) > end:
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
                if start + sj_1 < end:
                    green[i].append([start + sj_1, end])
                else:
                    green[i].append([end, end])
            else:
                if start + sj_1 + xj + clt < end:
                    green[i].append([start + sj_1 + xj + clt, end])
                else:
                    green[i].append([end, end])

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


# 整理绿灯相位，用于车辆轨迹生成
def regulargreen(signal, H, clt):
    green = [[], [], [], []]
    for c in signal.keys():
        sigc = signal[c]
        start = c * H
        for i in range(4):
            glist = sigc[i]
            for g in glist:
                if g[0] >= g[1]:
                    pass
                else:
                    rs, re = start + g[0], start + g[1]
                    green[i].append([rs, re])
    # 排序
    for g in green:
        for i in range(len(g) - 1):
            for j in range(len(g) - i - 1):
                if g[j][0] > g[j + 1][0]:
                    g[j], g[j + 1] = g[j + 1], g[j]
    # 合并
    Green = [[], [], [], []]
    for i in range(4):
        g = green[i]
        num = len(g)
        j = 1
        while j < num:
            if g[j-1][1] + clt == g[j][0]:
                Green[i].append([g[j-1][0], g[j][1]])
                j += 1
            else:
                Green[i].append([g[j-1][0], g[j-1][1]])
            j += 1

        if g[num-2][1] + clt == g[num-1][0]:
            pass
        else:
            Green[i].append([g[num-1][0], g[num-1][1]])
    return Green


# Function of location over time t
def locationt(init, p, tlist):
    x_t = []
    for t in tlist:
        (t0, v0) = (init[0], init[1])
        (tstart, vstart, xstart) = (t0, v0, 0)
        for i in range(len(p)):
            (a, start, end) = (p[i][0], p[i][1], p[i][2])
            if start <= t <= end:
                x_t.append(xstart + vstart * (t - tstart) + 0.5 * a * (t - tstart) ** 2)
                break
            else:
                xstart += vstart * (end - start) + 0.5 * a * (end - start) ** 2
                vstart += a * (end - start)
                tstart = end
    return x_t


# plot trajectories
def plotTra(platoon, P, L, green, T, clt, filename):
    fig, ax = plt.subplots(figsize=(12, 6))
    # 绘制绿灯信号
    for g in green:
        clock = np.arange(g[0], g[1], 0.2)
        y = np.ones_like(clock) + L
        ax.plot(clock, y, linestyle='solid', linewidth=3, color="green")
        clocky = np.arange(g[1], g[1] + clt, 0.2)
        yy = np.ones_like(clocky) + L
        ax.plot(clocky, yy, linestyle='solid', linewidth=3, color="yellow")
    # 绘制红灯信号
    clock1 = np.arange(0, green[0][0], 0.2)
    y = np.ones_like(clock1) + L
    ax.plot(clock1, y, linewidth=3, color="red")
    # ax.plot(clock1, y, linewidth=3, color="mistyrose")
    clock2 = np.arange(green[-1][1] + clt, T, 0.2)
    y = np.ones_like(clock2) + L
    ax.plot(clock2, y, linewidth=3, color="red")
    # ax.plot(clock2, y, linewidth=3, color="mistyrose")
    for i in range(0, len(green) - 1):
        clock = np.arange(green[i][1] + clt, green[i+1][0], 0.2)
        y = np.ones_like(clock) + L
        ax.plot(clock, y, linewidth=3, color="red")
        # ax.plot(clock, y, linewidth=3, color="mistyrose")

    for n in range(len(platoon)):
        initn = platoon[n].init
        pn = P[n]
        s, e = pn[0][1], pn[-1][2]
        tn = np.arange(s, e, 0.2)
        x_t = locationt(initn, pn, tn)
        if initn[3] == 1:
            ax.plot(tn, x_t, linewidth=1, color='darkorange')
            # ax.plot(tn, x_t, linestyle='dashed', linewidth=1, color='blue')
        else:
            ax.plot(tn, x_t, linewidth=1, color='blue')
            # ax.plot(tn, x_t, linestyle='solid', linewidth=1, color='blue')
    ax.set_ylim(bottom=0, top=L + 20)
    ax.set_xlim(left=0, right=T)
    ax.set_xlabel('Time (s)', fontsize=14, fontname='Times New Roman')
    ax.set_ylabel('Space (m)', fontsize=14, fontname='Times New Roman')
    labels = ax.get_xticklabels() + ax.get_yticklabels()
    [label.set_fontname('Times New Roman') for label in labels]
    [label.set_fontsize(14) for label in labels]
    plt.savefig(filename + ".png", dpi=600)
    # plt.show()
    plt.close(fig)


# Save the average travel time
def savetraveltime(P, scenario):
    if scenario == 0:
        filename = 'data/DLATraveltime.xlsx'
        workbook = xlsxwriter.Workbook(filename)
        sheet = workbook.add_worksheet('DLA')
    else:
        filename = 'data/FixedTraveltime.xlsx'
        workbook = xlsxwriter.Workbook(filename)
        sheet = workbook.add_worksheet('Fixed')
    sheet.write(0, 0, 'Movement')
    sheet.write(0, 1, 'Travel time')
    sheet.write(0, 2, 'Number')
    for i in range(16):
        sheet.write(i + 1, 0, i)
        mp = P[i]
        traveltime = []
        for p in mp:
            traveltime.append(p[-1][1] - p[0][1])
        average = np.mean(traveltime)
        vehnum = len(traveltime)
        sheet.write_number(i + 1, 1, average)
        sheet.write_number(i + 1, 2, vehnum)
    workbook.close()


# Save the travel time of individual vehicle
def vehicleTratime(P, scenario):
    if scenario == 0:
        filename = 'data/DLAvehTime.xlsx'
        workbook = xlsxwriter.Workbook(filename)
        sheet = workbook.add_worksheet('DLA')
    else:
        filename = 'data/FixedvehTime.xlsx'
        workbook = xlsxwriter.Workbook(filename)
        sheet = workbook.add_worksheet('Fixed')

    for i in range(16):
        sheet.write(0, i, i + 1)
        mp = P[i]
        for j in range(len(mp)):
            p = mp[j]
            traveltime = p[-1][1] - p[0][1]
            sheet.write(j + 1, i, traveltime)

    workbook.close()


# Save the signal timing plan
def savesignal(signal, scenario):
    if scenario == 0:
        filename = 'data/DLAsignal.xlsx'
        workbook = xlsxwriter.Workbook(filename)
        sheet = workbook.add_worksheet('DLA')
    else:
        filename = 'data/Fixedsignal.xlsx'
        workbook = xlsxwriter.Workbook(filename)
        sheet = workbook.add_worksheet('Fixed')
    # sheet.write(0, 0, 'Phase')
    for i in range(4):
        sheet.write(0, 3 * i, i + 1)
        sheet.write(1, 3 * i, 'Start')
        sheet.write(1, 3 * i + 1, 'End')
        sheet.write(1, 3 * i + 2, 'Duration')

    for i in range(4):
        green = signal[i]
        for j in range(len(green)):
            sheet.write(2 + j, 3 * i, green[j][0])
            sheet.write(2 + j, 3 * i + 1, green[j][1])
            sheet.write(2 + j, 3 * i + 2, green[j][1] - green[j][0])

    workbook.close()

