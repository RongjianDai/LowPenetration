# -*- coding: utf-8 -*-
"""
Created on Date 2022-01-14

@author: Rongjian Dai
"""
from Initialize import *

# Simulation parameters
(T, H, P) = (900, 90, 0.2)
L = 300
# 信号控制参数
(minG, clt, tstep) = (10, 3, 2)
# 轨迹控制参数
(a1, a2) = (2, -8)
# 跟驰参数
toff = (1, 2)
doff = (5.5, 6.5)
# 加速度折减系数
c = 0.9
# 交通流方向索引
move = ['0_1', '0_2', '0_3', '1_0', '1_2', '1_3', '2_0', '2_1', '2_3', '3_0', '3_1', '3_2']
# 限速
vmax = [[0, 12, 15, 13.5], [13.5, 0, 12, 15], [15, 13.5, 0, 12], [12, 15, 13.5, 0]]
# 基础交通需求
basicQ = [[0, 200, 400, 100],
          [150, 0, 150, 200],
          [380, 150, 0, 180],
          [100, 200, 100, 0]]
# 变化模式: vt[i]=0表示不变化； vt[i]=1表示arm i上左转和直行变化；vt[i]=2表示arm i上右转转和直行变化；vt[i]=3表示arm i上左转和右转变化
vt = [1, 0, 1, 0]
# 交通流量变化率, [4][4]列表
slope = qvariation(basicQ, T, vt)
# 计算专用直行车道ET的通行能力
sET = sat_ET(P, toff, doff)
# 各种类型车道饱和流量
# saturation = {'ET': 1621.62, 'EL': 1441.44, 'ER': 1297.3, 'LT': 1526.23, 'TR': 1441.44}
S = saturation(sET)
# 供给向量表, [6][6]列表, 排列模式如图13
supTab = supply(S)
# 实时交通流量矩阵，字典：key为控制周期，item为流量矩阵
QT = Qt(H, T, basicQ, slope)
# 优化各周期的车道功能, marking和u均为字典，key为周期
(marking, u) = lanemarking(QT)

# 给出车辆初始状态信息，entry为字典，key为move; value 每辆车初始状态的列表
entry = initialize(basicQ, slope, vmax, P, vt, T)
# for m in move:
#     print(m, ':')
#     print(entry[m])
#     print(len(entry[m]))


