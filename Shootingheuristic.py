# -*- coding: utf-8 -*-
"""
Created on Date 2022-01-17

@author: Rongjian Dai
"""

import matplotlib.pyplot as plt
import numpy as np
import W_R_data
from Vehicle import *


# Trajectory construction using the SH algorithm for a given movement
def construction(state, signal, a1, a2, L, H):
    P = []
    platoon = []
    toff = (1, 2)
    doff = (5.5, 6.5)
    for n in range(len(state)):
        platoon.append(Vehicle(a1, a2, toff, doff, n, state[n]))
    for n in range(len(platoon)):
        veh = platoon[n]
        if n == 0:
            veh.linit = None
            veh.lp = None
            print('第 0 辆：', veh.init)
            if veh.init[3] == 1:
                p = veh.SH(signal, L, H)
            else:
                p = veh.H_SH(signal, L, H)
            print('Tra：', p)
            P.append(p)
        else:
            veh.linit = platoon[n - 1].init
            veh.lp = P[n - 1]
            print('第', n, '辆：', veh.init)
            if veh.init[3] == 1:
                p = veh.SH(signal, L, H)
            else:
                p = veh.H_SH(signal, L, H)
            print('Tra：', p)
            # print('Its ps:', veh.ps)
            P.append(p)
    return P, platoon


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
def plotTra(platoon, P, L, green, H):
    fig, ax = plt.subplots()
    # 绘制绿灯信号
    for g in green:
        clock = np.arange(g[0], g[1], 0.1)
        y = np.ones_like(clock) + L
        ax.plot(clock, y, color="green", linewidth=3)
    # 绘制红灯信号
    clock = np.arange(0, green[0][0], 0.1)
    y = np.ones_like(clock) + L
    ax.plot(clock, y, color="red", linewidth=3)
    clock = np.arange(green[-1][1], H, 0.1)
    y = np.ones_like(clock) + L
    ax.plot(clock, y, color="red", linewidth=3)
    for i in range(0, len(green) - 1):
        clock = np.arange(green[i][1], green[i+1][0], 0.1)
        y = np.ones_like(clock) + L
        ax.plot(clock, y, color="red", linewidth=3)
    for n in range(len(platoon)):
        initn = platoon[n].init
        pn = P[n]
        s, e = pn[0][1], pn[-1][2]
        tn = np.arange(s, e, 0.1)
        x_t = locationt(initn, pn, tn)
        ax.plot(tn, x_t)
    ax.set_ylim(bottom=0, top=L + 20)
    ax.set_xlim(left=0)
    ax.set_xlabel('Time (s)', fontsize=12, fontname='Times New Roman')
    ax.set_ylabel('Space (m)', fontsize=12, fontname='Times New Roman')
    labels = ax.get_xticklabels() + ax.get_yticklabels()
    [label.set_fontname('Times New Roman') for label in labels]
    plt.show()


L = 300
H = 90
green = [[20, 40], [55, 75], [90, 105], [120, 140]]
(a1, a2) = (2, -8)
file = 'data\\InitialStates.xls'
state = W_R_data.readinit(file)
move = state[14][0:20]
print(move)
(P, platoon) = construction(move, green, a1, a2, L, H)
plotTra(platoon, P, L, green, H)


