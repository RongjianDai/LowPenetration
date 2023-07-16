# -*- coding: utf-8 -*-
"""
Created on 2021-12-14

@author: Rongjian Dai
"""

import gurobipy as gp
from gurobipy import GRB


def optimization(Q, SF):
    """
    :param Q: 交通需求
    :return: 控制策略，车道功能
    """
    # 模型参数
    # K = [1.125, 1.25]
    K = [[0, 1.25, 1, 1.125],
         [1.125, 0, 1.25, 1],
         [1, 1.125, 0, 1.25],
         [1.25, 1, 1.125, 0]]

    # 存储车道功能
    fun = [[[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]],
           [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]],
           [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]],
           [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]]
    multiplier = [0] * 4

    try:
        # 存储模型关系
        Art = [[1, 2], [2, 3], [3, 0], [0, 1]]  # Art[i] 为从arm i 出发的右转和直行对应的arm索引

        # create a model
        model = gp.Model("Lane assignment")

        # Create variables
        q = model.addVars(4, 4, 4, lb=0, ub=GRB.INFINITY, obj=0.0, vtype=GRB.CONTINUOUS, name="Assigned flow")
        l = model.addVars(4, 4, 4, lb=0, ub=1, obj=0.0, vtype=GRB.BINARY, name="Permitted")
        y = model.addVars(4, 4, lb=0, ub=1, obj=0.0, vtype=GRB.CONTINUOUS, name="Flow factor")
        u = model.addVars(4, lb=0, ub=10, obj=0.0, vtype=GRB.CONTINUOUS, name="Multiplier")
        R = model.addVars(4, 4, lb=0, ub=1, obj=0.0, vtype=GRB.BINARY, name="ER")

        # set objective function
        model.setObjective(gp.quicksum(u[i] for i in range(4)), GRB.MAXIMIZE)

        # add constraints
        # 1. Common multiplier
        for i in range(4):
            for j in range(4):
                if j != i:
                    model.addConstr(u[i] * Q[i][j] == gp.quicksum(q[i, j, k] for k in range(4)))
                else:
                    pass

        # 2. Each lane must be used
        for i in range(4):
            for k in range(4):
                laneuse = gp.LinExpr()
                for j in range(4):
                    laneuse += l[i, j, k]
                model.addConstr(laneuse >= 1)

        # 3. Uncommon lane-use patterns are not allowed
        for i in range(4):
            for k in range(4):
                laneuse = gp.LinExpr()
                for j in range(4):
                    laneuse += l[i, j, k]
                model.addConstr(laneuse <= 2)
                n = i - 1 if i - 1 >= 0 else i + 3
                m = i + 1 if i + 1 <= 3 else i - 3
                model.addConstr(l[i, n, k] + l[i, m, k] <= 1)

        # 4. To avoid potential conflicts within an arm
        m = [[3, 2], [0, 3], [1, 0], [2, 1]]
        for i in range(4):
            for j in range(4):
                if j != i:
                    for k in range(1, 4):
                        model.addConstrs(l[i, n, k - 1] <= 1 - l[i, j, k] for n in m[j])
                        model.addConstrs(l[i, n, k - 1] >= l[i, j, k] - 1 for n in m[j])
                else:
                    pass

        # 5. To avoid wasting of lane resources
        for i in range(4):
            for j in range(4):
                model.addConstr(1000 * Q[i][j] >= gp.quicksum(l[i, j, k] for k in range(4)))
                model.addConstrs(1000 * l[i, j, k] >= q[i, j, k] for k in range(4))

        # 6. Flow factor
        for i in range(4):
            for k in range(4):
                model.addConstr(SF * y[i, k] == gp.quicksum(K[i][j] * q[i, j, k] for j in range(4) if j != i))

        # 7. Relationship of flow factor
        for i in range(4):
            for k in range(3):
                model.addConstrs(y[i, k] - y[i, k+1] <= 1000 * (2 - l[i, j, k] - l[i, j, k+1]) for j in range(4) if j != i)
                model.addConstrs(y[i, k] - y[i, k+1] >= 1000 * (l[i, j, k] + l[i, j, k+1] - 2) for j in range(4) if j != i)

        # Generally, the following constraints don't work. If necessary, these constraints can be omitted
        # 8. Is an approach lane is an exclusive right turn lane
        for i in range(4):
            model.addConstrs(l[i, Art[i][0], k] - l[i, Art[i][1], k] <= 1000 * R[i, k] for k in range(4))
            model.addConstrs(l[i, Art[i][0], k] - l[i, Art[i][1], k] >= 1000 * (R[i, k] - 1) for k in range(4))

        # 9. To avoid merging potential conflicts
        for i in range(4):
            for j in Art[i]:
                m = j - 1 if j - 1 >= 0 else j + 3
                model.addConstr(4 - gp.quicksum(R[m, k] for k in range(4)) >= gp.quicksum(l[i, j, k] for k in range(4)))

        # Set the parameter
        # model.Params.method = 1
        model.Params.LogToConsole = False  # 显示求解过程
        # model.Params.MIPGap = 0.0001  # 百分比界差
        # model.Params.TimeLimit = 0.4  # 限制求解时间为 0.4 s

        # Optimize model
        model.optimize()

        # print("Largest multiplier = ", u.X)
        for i in range(4):
            multiplier[i] = u[i].X
        # print('u: ', multiplier)

        # Get the lane assignment scheme
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    fun[i][j][k] = int(l[i, j, k].X)

    except gp.GurobiError as e:
        print('Error code' + ': ' + str(e))

    except AttributeError:
        print('Encountered an attribute error')

    return fun, multiplier


# Q = [[0, 200, 400, 100], [150, 0, 150, 200], [380, 150, 0, 180], [100, 200, 100, 0]]
# turning = [[3, 2, 1], [0, 3, 2], [1, 0, 3], [2, 1, 0]]
# SF = 1636
# (fun, u) = optimization(Q, SF)
# for i in range(4):
#     print('Arm', i, '-- u: ', u[i])
#     for j in range(4):
#         if j != i:
#             for k in range(4):
#                 if fun[i][j][k] == 1:
#                     print('Lane', k, ': To arm', j)
