# -*- coding: utf-8 -*-
"""
Created on Date 2022-01-14

@author: Rongjian Dai
"""

from Scenario import *
from Setup import *
import Reoptimize
import W_R_data
import Movements
import copy


# 计算专用直行车道ET的通行能力
sET = sat_ET(P, toff, doff)
# 各种类型车道饱和流量
# saturation = {'ET': 1621.62, 'EL': 1441.44, 'ER': 1297.3, 'LT': 1526.23, 'TR': 1441.44}
S = saturation(sET)
# 供给向量表, [6][6][3]列表, 排列模式如图13
supTab = supply(S, schemeset)
# print(supTab)
usedQ = flowrate(basicQ, flowfactor)
# 交通流量变化率, [4][4]列表
slope = qvariation(turning, usedQ, T, vt)
# 实时交通流量矩阵，字典：key为控制周期，item为流量矩阵
QT = Qt(H, T, usedQ, slope)

# 优化各周期的车道功能, marking和u均为字典，key为周期
(marking, u) = lanemarking(QT, sET)
(Scheme, lanegroup, optc) = Reoptimize.reoptLA(QT, marking, supTab, schemeset, turning)
# print(lanegroup)
# print(u)
wave = [[False, False, False, True],
         [False, False, False, False],
         [False, False, False, False],
         [False, False, False, False]]
cyclen, gap = 80, 20
greentime = [[0, 10.61, 23.65, 23.65], [12.13, 0, 10.61, 12.13],
             [23.65, 23.65, 0, 10.61],[10.61, 12.13, 10.61, 0]]
# 给出车辆初始状态信息，entry为字典，key为move; value 每辆车初始状态的列表
entry = initialize(usedQ, slope, vmax, P, T, wave, gap, greentime, cyclen)
entry1 = copy.deepcopy(entry)
entry2 = copy.deepcopy(entry)
# 将车辆组成16个lane-level movements
initstate = Movements.movements(entry, Scheme, turning, H)
inittradition = Movements.tramovements(entry1, Scheme, turning, H)
initFixed = Movements.fixmovements(entry2, turning, H)
# Save the initial states of vehicles for each lane-level movement as a .xls file
filename = 'data/InitialStates.xls'
W_R_data.saveinit(initstate, filename)
# Fixed lane assignment
filename1 = 'data/TradInitialStates.xls'
W_R_data.saveinit(inittradition, filename1)
filename2 = 'data/FixedInitialStates.xls'
W_R_data.saveinit(initFixed, filename2)
# Save the lane assignment schemes in each control cycle as a .csv file
W_R_data.saveLA(lanegroup)



