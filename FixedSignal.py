# -*- coding: utf-8 -*-
"""
Created on Date 2023/7/14
@author: Rongjian Dai
"""
from Scenario import *
import W_R_data
import math
from Vehicle import *
import xlsxwriter
import Supplymethods


def timing(length):
    numcyc = math.ceil(T / length)
    signal = []
    gtime = [0, 0, 0, 0]
    armdemand = [780, 350, 400, 350]
    elen = length - clt * 4
    for i in range(4):
        gtime[i] = (armdemand[i] / sum(armdemand)) * elen
    p1, p2, p3, p4 = [], [], [], []
    for c in range(numcyc):
        S1 = c * length
        E1 = S1 + gtime[0]
        p1.append([S1, E1])
        S2 = E1 + clt
        E2 = S2 + gtime[1]
        p2.append([S2, E2])
        S3 = E2 + clt
        E3 = S3 + gtime[2]
        p3.append([S3, E3])
        S4 = E3 + clt
        E4 = S4 + gtime[3]
        p4.append([S4, E4])

    signal.append(p1)
    signal.append(p2)
    signal.append(p3)
    signal.append(p4)
    return signal


def trajectory(platoon, signal, T, Fixedm2p):
    P = []
    for i in range(16):
        phase = Fixedm2p[i]
        green = signal[phase]
        # print('move:', i, "Green:", green)
        pla = platoon[i]
        moveP = []
        for n in range(len(pla)):
            veh = pla[n]
            if n == 0:
                veh.linit = None
                veh.lp = None
                print('第 0 辆：', veh.init)
                p = veh.H_SH(green, L, T)
                print('Tra：', p)
                moveP.append(p)
            else:
                veh.linit = pla[n - 1].init
                veh.lp = moveP[n - 1]
                print('第', n, '辆：', veh.init)
                p = veh.H_SH(green, L, T)
                print('Tra：', p)
                moveP.append(p)

        P.append(moveP)
    return P


def savetraveltime(P):
    filename = 'data/FixedsignalTraveltime.xlsx'
    workbook = xlsxwriter.Workbook(filename)
    sheet = workbook.add_worksheet('Fixedsignal')
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
def showtrajectory(platoon, P, signal, folder, Fixedm2p):
    for i in range(16):
        mp = P[i]
        pla = platoon[i]
        phase = Fixedm2p[i]
        green = signal[phase]
        filename = folder + '/Movement' + str(i)
        Supplymethods.plotTra(pla, mp, L, green, T, clt, filename)


# 主函数
if __name__ == "__main__":
    siglength = 65
    Fixedm2p = {0: 1, 1: 0, 2: 0, 3: 0,
                4: 3, 5: 2, 6: 2, 7: 2,
                8: 1, 9: 0, 10: 0, 11: 0,
                12: 3, 13: 2, 14: 2, 15: 2}
    file = 'data/FixedInitialStates.xls'
    folder = 'figure/FixedSignal'
    signal = timing(siglength)
    print(signal)
    state = W_R_data.readinit(file)
    platoon = generation(state)
    P = trajectory(platoon, signal, T, Fixedm2p)
    savetraveltime(P)
    showtrajectory(platoon, P, signal, folder, Fixedm2p)