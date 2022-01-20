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
def construction(state, signal, a1, a2, L):
    P = []
    platoon = []
    (toff, doff) = (1, 5.5)
    for n in range(len(state)):
        platoon.append(Vehicle(a1, a2, toff, doff, n, state[n]))
    for n in range(len(platoon)):
        veh = platoon[n]
        if n == 0:
            veh.linit = None
            veh.lp = None
            p = veh.SH(signal, L)
            print('第0辆：', p)
            P.append(p)
        else:
            veh.linit = platoon[n - 1].init
            veh.lp = P[n - 1]
            p = veh.SH(signal, L)
            print('第', n, '辆：', p)
            print('Its ps:', veh.ps)
            P.append(p)
    return P, platoon


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
def plotTra(platoon, P, L, green):
    fig, ax = plt.subplots()
    # 绘制信号
    for g in green:
        clock = np.arange(g[0], g[1], 0.1)
        y = np.ones_like(clock) + L
        ax.plot(clock, y, color="green")
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
    plt.show()


L = 300
green = [[20, 40], [55, 75], [90, 105], [120, 140]]
(a1, a2) = (2, -8)
file = 'data\\InitialStates.xls'
state = W_R_data.readinit(file)
move = state[0][0:10]
print(move)
(P, platoon) = construction(move, green, a1, a2, L)
plotTra(platoon, P, L, green)




