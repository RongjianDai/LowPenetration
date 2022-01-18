# -*- coding: utf-8 -*-
"""
Created on Date 2022-01-17

@author: Rongjian Dai
"""

import numpy as np
from sympy import *


class Vehicle:
    def __init__(self, a1, a2, toff, doff, n, init):
        self.a1 = a1
        self.a2 = a2
        self.toff = toff
        self.doff = doff
        self.index = n
        self.init = init
        self.lp = None
        self.linit = None
        self.ps = []
        self.sinit = None

    # 给出安全边界 ps
    def shadow(self):
        (t0, v0) = (self.linit[0], self.linit[1])
        st0 = t0 + self.toff + self.doff / v0
        sX = self.locatspeed(self.linit, self.lp, (st0 - self.toff))
        self.sinit = [st0, sX[1]]
        for seg in self.lp:
            self.ps.append([seg[0], (seg[1] + self.toff), (seg[2] + self.toff)])

    # The SH algorithm for CAVs
    def SH(self, signal, L):
        p = []  # [number of segments][3] list: [][0] 加速度；[][1] 开始时间；[][2] 结束时间；
        (t0, v0, vmax) = (self.init[0], self.init[1], self.init[2])
        # Forward shooting process
        pf = []
        if self.lp is None:  # This vehicle is the first one，then it will arrive at the intersection in the fasteast way
            # The acclerating segment
            tvmax = t0 + (vmax - v0) / self.a1
            arrival = self.fastarrival(L)
            pf.append([self.a1, t0, tvmax])
            pf.append([0, tvmax, arrival])
        else:  # There is a preceding vehicle for this one
            # Get the safety boundary
            self.shadow()
            print('ps', self.ps)
            tvmax = t0 + (vmax - v0) / self.a1
            arrival = self.fastarrival(L)
            pf.append([self.a1, t0, tvmax])
            pf.append([0, tvmax, arrival])
            # 判断加速结束时刻是否会与安全边界相交
            selfX = self.locatspeed(self.init, pf, arrival)
            shadX = self.locatspeed(self.sinit, self.ps, arrival)
            print('ps', shadX[0], 'this', selfX[0])
            if shadX[0] > selfX[0]:  # 不相交，前向轨迹生成完毕
                print('不相交！')
                pass
            else:
                for s in range(len(pf)):
                    (ismerge, Ts, Tm) = self.merge(pf, pf[s], 1)
                    if ismerge:
                        pf[s][2] = Ts
                        pf.append([self.a2, Ts, Tm])
                        j = 0
                        while j < len(self.ps):
                            if self.ps[j][1] <= Tm < self.ps[j][2]:
                                pf.append([self.ps[j][0], Tm, self.ps[j][2]])
                                for x in range(j+1, len(self.ps)):
                                    pf.append(self.ps[x])
                                break
                            else:
                                continue
                            j += 1

                        break
                    else:
                        continue
        p = pf
        # Backward shooting process
        # Initial segment of pb
        # Merging segment
        return p

    # The modified SH algorithm for HVs
    def H_SH(self, signal, L):
        p = []

    # 计算相切点
    def merge(self, p, seg, stage):
        (ismerge, Ts, Tm) = (False, 0, 0)
        if stage == 1:  # Forward shooting
            # 相切前一段开始时间及速度
            selfX = self.locatspeed(self.init, p, seg[1])
            for ls in self.ps:   # ls为可能相切的ps部分
                if ls[2] < seg[1]:
                    continue
                else:
                    shadX = self.locatspeed(self.sinit, self.ps, ls[1])
                    (tn_1, tn) = (ls[1], seg[1])
                    (an_1, an) = (ls[0], seg[0])
                    (vn_1, vn) = (shadX[1], selfX[1])
                    (xn_1, xn) = (shadX[0], selfX[0])
                    (ts, tm) = (Symbol('ts'), Symbol('tm'))
                    eq1 = (an - self.a2) * ts + (self.a2 - an_1) * tm - (vn_1 - vn + an * tn - an_1 * tn_1)
                    eq2 = 0.5*(self.a2-an_1)*tm**2 - 0.5*(self.a2-an)*ts**2 + (an-self.a2)*ts*tm+(vn-an*tn+an_1*tn_1-vn_1)*tm+ \
                          (xn-xn_1+vn_1*tn_1-vn*tn+0.5*an*tn**2-0.5*an_1*tn_1**2)
                    variables = [ts, tm]
                    eqs = [eq1, eq2]
                    result = self.solve(eqs, variables)
                    if len(result) != 0 and result[0] >= seg[1] and result[1] <= ls[2]:
                        ismerge = True
                        (Ts, Tm) = (result[0], result[1])
                    else:
                        continue
        else:  # Backward shooting
            pass
        return ismerge, Ts, Tm

    # 求解方程组
    @staticmethod
    def solve(variables, eqs):
        result = solve(eqs, variables)
        return result

    # The location at the given time for a p and initial state
    @staticmethod
    def locatspeed(init, p, t):
        X = [0, 0]
        (t0, v0) = (init[0], init[1])
        (tstart, vstart, xstart) = (t0, v0, 0)
        for i in range(len(p)):
            (a, start, end) = (p[i][0], p[i][1], p[i][2])
            if start <= t <= end:
                X[0] = xstart + vstart * (t - tstart) + 0.5 * a * (t - tstart) ** 2
                X[1] = vstart + a * (t - tstart)
                break
            else:
                xstart = xstart + vstart * (end - start) + 0.5 * a * (end - start) ** 2
                vstart = vstart + a * (end - start)
                tstart = end
        return X

    # The possible earliest arrival time
    def fastarrival(self, L):
        (t0, v0, vmax) = (self.init[0], self.init[1], self.init[2])
        # 肯定能够加速到最大速度，因此不作判断，直接使用两段轨迹计算
        t1 = (vmax - v0) / self.a1
        t2 = (L - (vmax ** 2 - v0 ** 2) / (2 * self.a1)) / vmax
        return t0 + t1 + t2




