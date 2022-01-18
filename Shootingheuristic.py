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
            P.append(p)
    return P, platoon


# Plot the trajectory
def plotTra(P, init, L):
    for n in range(len(P)):
        (t0, v0, x0) = (init[n][0], init[n][1], 0)
        for seg in P[n]:
            t = np.arange(seg[1], seg[2], 0.1)
            x = x0 + v0 * (t - seg[1]) + 0.5 * seg[0] * (t - seg[1]) ** 2
            plt.plot(t, x)
            plt.xlim(0, 320)
            x0 += v0 * (seg[2] - t0) + 0.5 * seg[0] * (seg[2] - t0) ** 2
            v0 += seg[0] * (seg[2] - t0)
            t0 = seg[2]
    plt.show()



L = 300
signal = [[20, 40], [40, 55], [55, 75], [75, 90]]
(a1, a2) = (2, -8)
file = 'data\\InitialStates.xls'
state = W_R_data.readinit(file)
move = state[0][0:20]
(P, platoon) = construction(move, signal, a1, a2, L)
plotTra(P, move, L)




