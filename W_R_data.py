# -*- coding: utf-8 -*-
"""
Created on Date 2022-01-16

@author: Rongjian Dai
"""
import xlrd
import xlwt
import csv


# 车道功能存储
def saveLA(lanegroup):
    with open('data\\LA scheme.csv', 'w', newline='') as csvfile:
        fieldnames = ['Cycle', 0, 1, 2, 3]
        writer = csv.writer(csvfile)
        # 写入Arm index
        writer.writerow(fieldnames)
        for (c, la) in lanegroup.items():
            rlist = [c]
            for arm in range(4):
                rlist.append(la[arm])
            writer.writerow(rlist)
    csvfile.close()


# 车辆初始状态存储, 输入参数：initstate
def saveinit(initstate):
    filename = 'data\\InitialStates.xls'
    workbook = xlwt.Workbook()
    for arm in range(4):
        sheet = workbook.add_sheet('Arm' + str(arm))
        sheet.write(0, 0, 'Index')
        for k in range(4):
            sheet.write(0, 4 * k + 1, k)
            sheet.write(1, 4 * k + 1, 't0')
            sheet.write(1, 4 * k + 2, 'v0')
            sheet.write(1, 4 * k + 3, 'vmax')
            sheet.write(1, 4 * k + 4, 'type')
        max = 0
        for lane in range(4 * arm, 4 * arm + 4):
            move = initstate[lane]
            vehnum = len(move)
            max = vehnum if vehnum > max else max
            scol = 4 * (lane - 4 * arm) + 1
            for n in range(vehnum):
                for j in range(4):
                    sheet.write(n + 2, scol + j, move[n][j])
        for n in range(max):
            sheet.write(n + 2, 0, n + 1)

    workbook.save(filename)


# 读取车辆初始状态，返回列表[16][number of vehicles][3]
def readinit(file):
    data = xlrd.open_workbook(file)
    init = []
    for arm in range(4):
        sheet = data.sheet_by_index(arm)
        for lane in range(4):
            scol = 4 * lane + 1
            state = []
            t0 = sheet.col_values(scol)
            v0 = sheet.col_values(scol + 1)
            vmax = sheet.col_values(scol + 2)
            type = sheet.col_values(scol + 3)
            for j in range(2, len(t0)):
                if t0[j] != '':
                    state.append([t0[j], v0[j], vmax[j], type[j]])
            init.append(state)

    return init

