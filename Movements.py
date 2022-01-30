# -*- coding: utf-8 -*-
"""
Created on Date 2022-01-14

@author: Rongjian Dai
"""


# Lane-level movement
def movements(entry, Scheme, turning, H):
    initstate = {}
    for i in range(4):
        (lj, tj, rj) = (turning[i][0], turning[i][1], turning[i][2])
        # 取出和lane-level movement i 对应的初始状态
        ltkey = str(i) + '_' + str(lj)
        thkey = str(i) + '_' + str(tj)
        rtkey = str(i) + '_' + str(rj)
        ltmove = entry[ltkey]
        thmove = entry[thkey]
        rtmove = entry[rtkey]
        lanelevel = [[], [], [], []]
        lastbir = [0, 0, 0, 0]
        moves = [ltmove, thmove, rtmove]
        turn = [lj, tj, rj]
        c = 0
        while True:
            # 该Arm上所有车辆分配完成后结束循环
            if len(ltmove) + len(thmove) + len(rtmove) == 0:
                break
            end = c * H + H
            lanes = availablelane(i, turn, Scheme[c])     # [3][]list [m][] 表示转向m可以使用的车道集合
            while True:
                # 按时间顺序依次分配到各车道
                ltbir = ltmove[0][0] if len(ltmove) != 0 else 40000
                thbir = thmove[0][0] if len(thmove) != 0 else 40000
                rtbir = rtmove[0][0] if len(rtmove) != 0 else 40000
                birth = [ltbir, thbir, rtbir]
                first = 0
                for _ in range(3):
                    if birth[_] < birth[first]:
                        first = _
                    else:
                        continue
                if birth[first] > end:    # 若最早的已经不在该周期，则进入下一周期
                    c += 1
                    break
                else:
                    veh = moves[first].pop(0)
                    usedlane = lanes[first]
                    if len(usedlane) == 1:  # 若只有一条车道可用，直接分配
                        thislane = usedlane[0]
                        lanelevel[thislane].append(veh)
                        lastbir[thislane] = birth[first]
                    else:  # 若有多条车道可选，则选择和上一辆车车头时距最大的那个
                        last = usedlane[0]
                        for k in usedlane:
                            if lastbir[k] < lastbir[last]:
                                last = k
                        lanelevel[last].append(veh)
                        lastbir[last] = birth[first]

        initstate[i * 4] = lanelevel[0]
        initstate[i * 4 + 1] = lanelevel[1]
        initstate[i * 4 + 2] = lanelevel[2]
        initstate[i * 4 + 3] = lanelevel[3]

    return initstate


# 给定movement(i,j)在周期c内的可用车道集合
def availablelane(oarm, turn, Scheme):
    lanes = []
    scheme = Scheme[oarm]
    for j in turn:
        l = []
        for k in range(4):
            if scheme[j][k] == 1:
                l.append(k)
            else:
                continue
        lanes.append(l)
    return lanes


# 传统模式下的车辆分配
def tramovements(entry, Scheme, turning, H):
    initstate = {}
    for i in range(4):
        (lj, tj, rj) = (turning[i][0], turning[i][1], turning[i][2])
        # 取出和lane-level movement i 对应的初始状态
        ltkey = str(i) + '_' + str(lj)
        thkey = str(i) + '_' + str(tj)
        rtkey = str(i) + '_' + str(rj)
        ltmove = entry[ltkey]
        thmove = entry[thkey]
        rtmove = entry[rtkey]
        lanelevel = [[], [], [], []]
        lastbir = [0, 0, 0, 0]
        moves = [ltmove, thmove, rtmove]
        turn = [lj, tj, rj]
        c = 0
        while True:
            # 该Arm上所有车辆分配完成后结束循环
            if len(ltmove) + len(thmove) + len(rtmove) == 0:
                break
            end = c * H + H
            lanes = availablelane(i, turn, Scheme[0])  # [3][]list [m][] 表示转向m可以使用的车道集合
            # print('lanes:', lanes)
            while True:
                # 按时间顺序依次分配到各车道
                ltbir = ltmove[0][0] if len(ltmove) != 0 else 40000
                thbir = thmove[0][0] if len(thmove) != 0 else 40000
                rtbir = rtmove[0][0] if len(rtmove) != 0 else 40000
                birth = [ltbir, thbir, rtbir]
                first = 0
                for _ in range(3):
                    if birth[_] < birth[first]:
                        first = _
                    else:
                        continue
                if birth[first] > end:  # 若最早的已经不在该周期，则进入下一周期
                    c += 1
                    break
                else:
                    veh = moves[first].pop(0)
                    usedlane = lanes[first]
                    if len(usedlane) == 1:  # 若只有一条车道可用，直接分配
                        thislane = usedlane[0]
                        lanelevel[thislane].append(veh)
                        lastbir[thislane] = birth[first]
                    else:  # 若有多条车道可选，则选择和上一辆车车头时距最大的那个
                        last = usedlane[0]
                        for k in usedlane:
                            if lastbir[k] < lastbir[last]:
                                last = k
                        lanelevel[last].append(veh)
                        lastbir[last] = birth[first]

        initstate[i * 4] = lanelevel[0]
        initstate[i * 4 + 1] = lanelevel[1]
        initstate[i * 4 + 2] = lanelevel[2]
        initstate[i * 4 + 3] = lanelevel[3]

    return initstate
