import numpy as np
from CPG.computeOffsets import computeOffsets
from Tools.constrainSE3 import *
from CPG.limitValue import *
from CPG.groundIK import *
from CPG.supelliptRot import *
import copy
import rospy
import math
from matplotlib import pyplot as plt

pi = np.pi


def CPG(cpg, t, dt):
    np.set_printoptions(precision=5)

    set_lastang = np.array([-pi / 2, pi / 2, -pi / 2, pi / 2, -pi / 2, pi / 2])

    shoulders1 = range(0, 18, 3)  # joint IDs of the shoulders
    shoulders2 = range(1, 18, 3)  # joint IDs of the second shoulder joints
    elbows = range(2, 3, 18)  # joint IDs of the elbow joints

    # CPG Implmentation/Equations
    gamma = 3  # forcing to limit cycle (larger = stronger)
    lambdaa = 1  # coupling strength (larger = weaker)

    if cpg['move']:

        cpg['w'] = 10
        # print('a',cpg['a'])
        # print('b', cpg['b'])
        # print('w', cpg['w'])
        # print('y', cpg['y'][t, :])
        # print('cy', cpg['cy'])
        # print('x', cpg['x'][t, :])
        # print('cx', cpg['cx'])
        dx = -1 * -1 * ((cpg['a'] * cpg['b']) / 2) * cpg['w'] * 2 * ((cpg['y'][t, :] - cpg['cy']) / (cpg['b'] ** 2)) + 1 * (
                    1 - (((cpg['x'][t, :] - cpg['cx']) / cpg['a']) ** 2) - ((cpg['y'][t, :] - cpg['cy']) / cpg['b']) ** 2) * 2 * (
                     (cpg['x'][t, :] - cpg['cx']) / (cpg['a'] ** 2))
        dy = -1 * ((cpg['a'] * cpg['b']) / 2) * cpg['w'] * 2 * ((cpg['x'][t, :] - cpg['cx']) / (cpg['a'] ** 2)) + 1 * (
                    1 - (((cpg['x'][t, :] - cpg['cx']) / cpg['a']) ** 2) - ((cpg['y'][t, :] - cpg['cy']) / cpg['b']) ** 2) * 2 * (
                     (cpg['y'][t, :] - cpg['cy']) / (cpg['b'] ** 2)) + (np.dot(0.3 * cpg['K'], (cpg['y'][t, :] - cpg['cy'])))
        #
        # x = x + 0.02 * f1
        # y = y + 0.02 * f2
        # x_output = x.copy()
        # y_output = y.copy()


        for value in range(6):
            truth = (cpg['CPGStanceDelta'].flatten())[value]
            if truth:
                cpg['xGr'][value] = cpg['x'][t, value]

        x_pos = abs(cpg['nomY'] - cpg['s1OffsetY'])

        dx_fact = 1 / (x_pos * np.reciprocal(np.cos(cpg['s1OffsetAngY'] + cpg['legs'][0, shoulders1])) ** 2)

        dx_const = -1 * cpg['scaling'] * cpg['desired_speed'] * dx_fact
        dx_const = dx[0, :]
        truther = False
        updStab = np.logical_or(cpg['CPGStance'], (dy[0] < 0))

    else:
        dx = 0
        dy = 0
        truther = True
        dx_const = 0
        updStab = np.logical_or(cpg['CPGStance'], np.array([False, False, False, False, False, False]))

    # test_ang = np.array([0, 0, -1.57,
    #                      0, 0, 1.57,
    #                      0, 0, -1.57,
    #                      0, 0, 1.57,
    #                      0, 0, -1.57,
    #                      0, 0, 1.57])
    # test_ang = np.reshape(test_ang[0:18], [1, 18])
    #
    # test_positions = cpg['xmk'].getLegPositions(test_ang)
    # print(test_positions)
    # [[0.51611  0.51611  0.0575   0.0575 - 0.45861 - 0.45861]
    #  [0.23158 - 0.23158  0.51276 - 0.51276  0.33118 - 0.33118]
    # [-0.2249 - 0.2249 - 0.2249 - 0.2249 - 0.2249 - 0.2249]]
    # ang = cpg['xmk'].getLegIK(test_positions)
    # print(ang)

    # Integrate dx & dy to produce joint commands


    cpg['x'][t + 1, :] = cpg['x'][t, :] + dx * dt
    cpg['xGr'] = cpg['xGr'] + dx_const * dt
    cpg['y'][t + 1, :] = cpg['y'][t, :] + dy * dt
    # print(cpg['x'][t + 1, :])

    # cpg['x'][t+1,:] = -cpg['x'][t,:]
    # cpg['xGr'] = -cpg['xGr']

    # Command CPG-generated values to joints
    yOut = cpg['y'][t + 1, :]
    xOut = np.zeros([1, 6])

    SignBack = -1 if (cpg['direction'] == 'backwards') else 1
    SignLeft = -1 if (cpg['direction'] == 'left') else 1
    SignRight = -1 if (cpg['direction'] == 'right') else 1

    for value in range(6):
        truth = cpg['CPGStance'][value]
        if truth:
            xOut[:, value] = cpg['xGr'][value]
        else:
            if value % 2 == 0:
                xOut[:, value] = SignLeft * SignBack * cpg['x'][t + 1, value]
            else:
                xOut[:, value] = SignRight * SignBack * cpg['x'][t + 1, value]

    # cpg['legs'][0,0:18:3] = limitValue((cpg['nomOffset'][0,:] + cpg['shouldersCorr'] * xOut), pi/2 * cpg['scaling'])
    # cpg['legs'][0,1:19:3] = cpg['nomOffset'][1,:] + cpg['shouldersCorr'] *  np.maximum(0, yOut)
    # print('x', cpg['nomOffset'])
    cpg['legs'][0, 0:18:3] = limitValue((cpg['shouldersCorr'] * xOut),
                                        pi / 2 * cpg['scaling'])  # x  make sure x is in the range of [-pi/2, pi/2]
    cpg['legs'][0, 1:19:3] = cpg['shouldersCorr'] * np.maximum(0, yOut)  # y
    # print(cpg['legs'][0, 0:18:3])
    # JOINT 3 - FOR WALKING TRIALS
    cpg['legs'][0, 0:18:3] = cpg['legs'][0, 0:18:3] #/ cpg['scaling']
    # print(cpg['a'])
    cpg['legs'][0, 1:19:3] = cpg['legs'][0, 1:19:3] #/ cpg['scaling']

    I = (np.logical_or(cpg['CPGStance'], dy < 0))  # the leg on the ground

    dist = cpg['nomY']  # coordinate(y) of the end effector of 6 legs

    indicies = []
    for index in range(6):
        if truther:  # moving : truther = false
            truth = I[index]
        else:
            truth = I[0, index]
        if truth:
            indicies.append(index)

    # angs = groundIK(cpg, dist, indicies)
    #
    # for index in indicies:
    #     cpg['legs'][0, 2+index*3] = angs[2+index*3]

    Leglength = 0.325
    z1 = -math.pi / 2 + math.asin((Leglength * math.cos(math.pi / 3 + cpg['a'][0][0]/2) / (
        math.cos(math.pi / 3 - cpg['x'][t + 1, :][0] * (-1))) - Leglength) / Leglength)
    z2 = math.pi / 2 - math.asin(
        (Leglength * math.cos(math.pi / 3 + cpg['a'][0][1]/2) / (math.cos(math.pi / 3 + cpg['x'][t + 1, :][1]))
         - Leglength) / Leglength)
    z3 = -math.pi / 2 + abs(math.asin((Leglength / math.cos(cpg['x'][t + 1, :][3]) - Leglength) / Leglength))
    z4 = math.pi / 2 - abs(math.asin((Leglength / math.cos(cpg['x'][t + 1, :][2] * (-1)) - Leglength) / Leglength))
    z5 = -math.pi / 2 + math.asin((Leglength * math.cos(math.pi / 3 + cpg['a'][0][4]/2) / (
        math.cos(math.pi / 3 + cpg['x'][t + 1, :][4] * (-1))) - Leglength) / Leglength)
    z6 = math.pi / 2 - math.asin((Leglength * math.cos(math.pi / 3 + cpg['a'][0][5]/2) / (
        math.cos(math.pi / 3 - cpg['x'][t + 1, :][5])) - Leglength) / Leglength)
    # print(cpg['a'][0][0])
    # print(cpg['x'][t + 1, :][0])
    # print(cpg['x'][t + 1, :])
    cpg['legs'][0, 2] = z1
    cpg['legs'][0, 5] = z2
    cpg['legs'][0, 8] = z3
    cpg['legs'][0, 11] = z4
    cpg['legs'][0, 14] = z5
    cpg['legs'][0, 17] = z6

    cpg['legs'] = np.reshape(cpg['legs'][0:18], [1, 18])

    # legs = np.copy(cpg['legs'])
    # print(legs)

    positions = cpg['xmk'].getLegPositions(cpg['legs'])
    print(cpg['legs'])

    return cpg,positions
