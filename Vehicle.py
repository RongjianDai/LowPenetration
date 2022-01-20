# -*- coding: utf-8 -*-
"""
Created on Date 2022-01-17

@author: Rongjian Dai
"""

import numpy as np
import sympy
import matplotlib.pyplot as plt


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

    # 返回给定轨迹和初始状态，在t时刻的位置和速度
    @staticmethod
    def locspeed(init, p, t):
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
                xstart += vstart * (end - start) + 0.5 * a * (end - start) ** 2
                vstart += a * (end - start)
                tstart = end
        return X

    # 求解方程组
    @staticmethod
    def solver(eqs, variables):
        result = sympy.solve(eqs, variables, dict=True)
        return result

    # The shadow trajectory
    def shadow(self):
        lp = self.lp
        (t0, v0) = (self.linit[0], self.linit[1])
        st0 = t0 + self.toff + self.doff / v0
        sX = self.locspeed(self.linit, self.lp, (st0 - self.toff))
        self.sinit = [st0, sX[1]]
        for i in range(len(lp)):
            if (lp[i][1] + self.toff) > st0:
                self.ps.append([lp[i - 1][0], st0, (lp[i][1] + self.toff)])
                for j in range(i, len(lp)):
                    self.ps.append([self.lp[j][0], (self.lp[j][1] + self.toff), (self.lp[j][2] + self.toff)])
                break

    # The possible earliest arrival time
    def fastarrival(self, L):
        (t0, v0, vmax) = (self.init[0], self.init[1], self.init[2])
        # 肯定能够加速到最大速度，因此不作判断，直接使用两段轨迹计算
        t1 = (vmax - v0) / self.a1
        t2 = (L - (vmax ** 2 - v0 ** 2) / (2 * self.a1)) / vmax
        return t0 + t1 + t2

    def time2x(self, init, p, x):
        for i in range(len(p)):
            o = self.locspeed(init, p, p[i][1])
            d = self.locspeed(init, p, p[i][2])
            if o[0] <= x < d[0]:
                t = sympy.Symbol('t', real=True)
                eq = o[0] + o[1] * t + 0.5 * p[i][0] * t ** 2 - x
                result = sympy.solve(eq)
                for r in result:
                    if 0 <= r < p[i][2] - p[i][1]:
                        p2x = r + p[i][1]
                        return p2x
                    else:
                        pass
                break
            else:
                continue

    # Determining whether a backward shooting process is needed
    def needbackward(self, pf, green, L):
        needBSP = True
        expectarrive = 80
        arrival = self.time2x(self.init, pf, L)
        for g in green:
            if g[0] <= arrival <= g[1]:
                needBSP = False
                break
            else:
                continue
        if needBSP:
            # Get the expected arrival time for this CAV
            for i in range(len(green)):
                if arrival < green[i][0]:
                    expectarrive = green[i][0]
                    break
                else:
                    continue
        return needBSP, expectarrive

    # 计算相切点
    def forwardMerge(self, pf, seg):
        ismerge, Ts, Tm, whichps = False, 0, 0, 0
        # 相切前一段开始时间，初始位置，初始速度及加速度
        selfX = self.locspeed(self.init, pf, seg[1])
        tn, an, vn, xn = seg[1], seg[0], selfX[1], selfX[0]
        for i in range(len(self.ps)):
            psseg = self.ps[i]
            if tn > psseg[2]:
                continue
            else:
                shadX = self.locspeed(self.sinit, self.ps, psseg[1])
                tn_1, an_1, vn_1, xn_1 = psseg[1], psseg[0], shadX[1], shadX[0]
                (ts, tm) = (sympy.Symbol('ts', real=True), sympy.Symbol('tm', real=True))
                eq1 = vn + an * (ts - tn) + self.a2 * (tm - ts) - vn_1 - an_1 * (tm - tn_1)
                eq2 = xn + vn * (ts - tn) + 0.5 * an * (ts - tn) ** 2 + (vn + an * (ts - tn)) * (tm - ts) + \
                    0.5 * self.a2 * (tm - ts) ** 2 - xn_1 - vn_1 * (tm - tn_1) - 0.5 * an_1 * (tm - tn_1) ** 2
                variables = [ts, tm]
                eqs = [eq1, eq2]
                result = self.solver(eqs, variables)
                if len(result) == 0:
                    pass
                elif len(result) == 1:
                    print('result:', result)
                    ts, tm = result[0][ts], result[0][tm]
                    if 0 <= ts < tm:
                        if tn <= ts < seg[2] and tn_1 <= tm <= psseg[2]:
                            ismerge = True
                            (Ts, Tm) = (ts, tm)
                            whichps = i
                            return ismerge, Ts, Tm, whichps
                    else:
                        pass
                else:
                    print('result:', result)
                    for r in result:
                        rts, rtm = r[ts], r[tm]
                        if 0 <= rts < rtm:
                            if tn <= rts <= seg[2] and tn_1 <= rtm <= psseg[2]:
                                ismerge = True
                                (Ts, Tm) = (rts, rtm)
                                whichps = i
                                return ismerge, Ts, Tm, whichps
                        else:
                            continue
        return ismerge, Ts, Tm, whichps

    # 计算backward shooting 的相切点
    def backwardMerge(self, pf, which, fseg, L, arrival):
        ismerge, Ts, Tm= False, 0, 0
        t0vmax = self.init[2] / self.a1
        xstop = L - self.init[2] ** 2 / (2 * self.a1)
        an, tn = fseg[0], fseg[1]
        selfX = self.locspeed(self.init, pf, tn)
        xn, vn = selfX[0], selfX[1]
        if which == 1:  # Stop segment
            (ts, tm) = (sympy.Symbol('ts', real=True), sympy.Symbol('tm', real=True))
            eq1 = vn + an * (ts - tn) + self.a2 * (tm - ts)
            eq2 = xn + vn * (ts - tn) + 0.5 * an * (ts - tn) ** 2 + (vn + an * (ts - tn)) * (tm - ts) + \
                  0.5 * self.a2 * (tm - ts) ** 2 - xstop
            variables = [ts, tm]
            eqs = [eq1, eq2]
            result = self.solver(eqs, variables)
            if len(result) == 0:
                pass
            elif len(result) == 1:
                print('result:', result)
                ts, tm = result[0][ts], result[0][tm]
                if 0 <= ts < tm:
                    if tn < ts <= fseg[2] and tm < (arrival - t0vmax):
                        ismerge = True
                        (Ts, Tm) = (ts, tm)
                    else:
                        pass
                else:
                    pass
            else:
                print('result:', result)
                for r in result:
                    rts, rtm = r[ts], r[tm]
                    if 0 <= rts < rtm:
                        if tn < rts <= fseg[2] and rtm < (arrival - t0vmax):
                            ismerge = True
                            (Ts, Tm) = (rts, rtm)
                            break
                    else:
                        continue
        else:  # Accelerating segment
            vmax = self.init[2]
            (ts, tm) = (sympy.Symbol('ts', real=True), sympy.Symbol('tm', real=True))
            eq1 = vn + an * (ts - tn) + self.a2 * (tm - ts) - vmax + self.a1 * (arrival - tm)
            eq2 = xn + vn * (ts - tn) + 0.5 * an * (ts - tn) ** 2 + (vn + an * (ts - tn)) * (tm - ts) + \
                  0.5 * self.a2 * (tm - ts) ** 2 - L + (vmax ** 2 - (vmax - self.a1 * (arrival - tm)) ** 2) / (
                              2 * self.a1)
            variables = [ts, tm]
            eqs = [eq1, eq2]
            result = self.solver(eqs, variables)
            if len(result) == 0:
                pass
            elif len(result) == 1:
                print('result:', result)
                ts, tm = result[0][ts], result[0][tm]
                if 0 <= ts < tm:
                    if tn < ts <= fseg[2] and tm < (arrival - t0vmax):
                        ismerge = True
                        (Ts, Tm) = (ts, tm)
                    else:
                        pass
                else:
                    pass
            else:
                print('result:', result)
                for r in result:
                    rts, rtm = r[ts], r[tm]
                    if 0 <= rts < rtm:
                        if tn < rts <= fseg[2] and rtm < (arrival - t0vmax):
                            ismerge = True
                            (Ts, Tm) = (rts, rtm)
                            break
                    else:
                        continue

        return ismerge, Ts, Tm

    # The SH algorithm for CAVs
    def SH(self, green, L):
        p = []  # [number of segments][3] list: [][0] 加速度；[][1] 开始时间；[][2] 结束时间；
        # Forward shooting process
        pf = []
        if self.lp is None:  # This vehicle is the first one，then it will arrive at the intersection in the fasteast way
            (t0, v0, vmax) = (self.init[0], self.init[1], self.init[2])
            # The acclerating segment
            tvmax = t0 + (vmax - v0) / self.a1
            arrival = self.fastarrival(L)
            pf.append([self.a1, t0, tvmax])
            pf.append([0, tvmax, arrival])
            pf.append([0, arrival, arrival + 10])  # 延长pf10s
        else:  # There is a preceding vehicle for this one
            # Get the safety boundary
            self.shadow()
            # 判断初始时间是否符合要求，不符合需要修正
            if self.init[0] >= self.sinit[0]:
                pass
            else:
                self.init[0] = self.sinit[0]
            (t0, v0, vmax) = (self.init[0], self.init[1], self.init[2])
            tvmax = t0 + (vmax - v0) / self.a1
            arrival = self.fastarrival(L)
            pf.append([self.a1, t0, tvmax])
            pf.append([0, tvmax, arrival])
            pf.append([0, arrival, arrival + 10])  # 延长pf10s
            # 判断加速结束时刻是否会与安全边界相交
            selft2L = self.time2x(self.init, pf, L)
            print('selft2L:', selft2L)
            shadt2L = self.time2x(self.sinit, self.ps, L)
            print('shadt2L:', shadt2L)
            if selft2L >= shadt2L:  # 不相交，前向轨迹生成完毕
                pass
            else:  # 相交需要求解merging segment
                for i in range(len(pf)):
                    seg = pf[i]
                    ismerge, Ts, Tm, whichps = self.forwardMerge(pf, seg)
                    if ismerge:
                        pf = pf[0:i]  # 清除 i-1 之后的轨迹段
                        pf.append([seg[0], seg[1], Ts])
                        pf.append([self.a2, Ts, Tm])  # Merging segment
                        pf.append([self.ps[whichps][0], Tm, self.ps[whichps][2]])  # 第一段紧密跟随轨迹
                        for j in range(whichps + 1, len(self.ps)):
                            pf.append(self.ps[j])
                        break
                    else:
                        continue
                pass

        p = pf
        # Backward shooting process
        needBSP, expectarrive = self.needbackward(pf, green, L)
        if needBSP:  # 需要BSP
            t0vmax = self.init[2] / self.a1
            for i in range(len(p)):
                seg = p[i]
                ismerge, Ts, Tm = self.backwardMerge(pf, 1, seg, L, expectarrive)
                if ismerge:
                    p = p[0:i]  # 清除 i-1 之后的轨迹段
                    p.append([seg[0], seg[1], Ts])
                    p.append([self.a2, Ts, Tm])
                    p.append([0, Tm, expectarrive - t0vmax])
                    p.append([self.a1, expectarrive - t0vmax, expectarrive])
                    p.append([0, expectarrive, expectarrive + 10])
                    break
                else:
                    ismerge, Ts, Tm = self.backwardMerge(pf, 2, seg, L, expectarrive)
                    if ismerge:
                        p = p[0:i]  # 清除 i-1 之后的轨迹段
                        p.append([seg[0], seg[1], Ts])
                        p.append([self.a2, Ts, Tm])
                        p.append([self.a1, Tm, expectarrive])
                        p.append([0, expectarrive, expectarrive + 10])
        return p

    # The SH algorithm for human-driven vehicles
    def H_SH(self, green, L):
        p = []
        # Forward shooting process
        pf = []
        if self.lp is None:  # This vehicle is the first one，then it will arrive at the intersection in the fasteast way
            (t0, v0, vmax) = (self.init[0], self.init[1], self.init[2])
            # The acclerating segment
            tvmax = t0 + (vmax - v0) / self.a1
            arrival = self.fastarrival(L)
            pf.append([self.a1, t0, tvmax])
            pf.append([0, tvmax, arrival])
            pf.append([0, arrival, arrival + 10])  # 延长pf10s
        else:  # There is a preceding vehicle for this one
            # Get the safety boundary
            self.shadow()
            # 判断初始时间是否符合要求，不符合需要修正
            if self.init[0] >= self.sinit[0]:
                pass
            else:
                self.init[0] = self.sinit[0]
            (t0, v0, vmax) = (self.init[0], self.init[1], self.init[2])
            tvmax = t0 + (vmax - v0) / self.a1
            arrival = self.fastarrival(L)
            pf.append([self.a1, t0, tvmax])
            pf.append([0, tvmax, arrival])
            pf.append([0, arrival, arrival + 10])  # 延长pf10s
            # 判断加速结束时刻是否会与安全边界相交
            selft2L = self.time2x(self.init, pf, L)
            print('selft2L:', selft2L)
            shadt2L = self.time2x(self.sinit, self.ps, L)
            print('shadt2L:', shadt2L)
            if selft2L >= shadt2L:  # 不相交，前向轨迹生成完毕
                pass
            else:  # 相交需要求解merging segment
                for i in range(len(pf)):
                    seg = pf[i]
                    ismerge, Ts, Tm, whichps = self.forwardMerge(pf, seg)
                    if ismerge:
                        pf = pf[0:i]  # 清除 i-1 之后的轨迹段
                        pf.append([seg[0], seg[1], Ts])
                        pf.append([self.a2, Ts, Tm])  # Merging segment
                        pf.append([self.ps[whichps][0], Tm, self.ps[whichps][2]])  # 第一段紧密跟随轨迹
                        for j in range(whichps + 1, len(self.ps)):
                            pf.append(self.ps[j])
                        break
                    else:
                        continue
                pass

        p = pf
        # Backward shooting process
        needBSP, expectarrive = self.needbackward(pf, green, L)
        if needBSP:
            for i in range(len(p)):
                seg = p[i]

        else:
            pass
        return p

    # Backward merging for H_SH
    def H_BSP(self, pf, which, fseg, L, arrival):
        ismerge, Ts, Tm = False, 0, 0
        
        return ismerge, Ts, Tm
