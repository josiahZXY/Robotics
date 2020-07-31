import numpy as np
from CPG.computeOffsets import computeOffsets
from Tools.constrainSE3 import *
from CPG.limitValue import *
from CPG.groundIK import * 
from CPG.supelliptRot import *
import copy



pi = np.pi

def CPG(cpg, t, dt):
    np.set_printoptions(precision=5)

    set_lastang = np.array([-pi/2,pi/2,-pi/2,pi/2,-pi/2,pi/2])

    shoulders1 = range(0,18,3) #joint IDs of the shoulders
    shoulders2 = range(1,18,3) #joint IDs of the second shoulder joints
    elbows = range(2,3,18) #joint IDs of the elbow joints

    dynOffsetError = copy.deepcopy(np.zeros([3,6]));

    if t > cpg['initLength']:
        [dynOffsetError,cpg] = computeOffsets(cpg, t, dt);

    #Scale
    dynOffsetError = dynOffsetError * cpg['scaling'] * 0

    # CPG Implmentation/Equations

    gamma = 3 #forcing to limit cycle (larger = stronger)
    lambdaa = 1 #coupling strength (larger = weaker)

    #Determine which legs are in air
    x0 = cpg['dynOffset'][0,:]
    y0 =  cpg['dynOffset'][1,:]

    cpg['x0'] = -x0;
    cpg['x'] = -cpg['x'] ;

    cpg['y0'] = y0;

    if cpg['move']:
        tauH = 100; #sharpness of transition
        cpg['w'] = cpg['wSwing']

        H = (abs((cpg['x'][t,:] - x0)/(cpg['a']))**cpg['n'] + abs((cpg['y'][t,:] - y0)/(cpg['b']))**cpg['n']);
        dHdx = (cpg['n'] * np.sign(cpg['x'][t,:] - x0) * (abs((cpg['x'][t,:] - x0)/cpg['a'])**(cpg['n']-1)))/ (cpg['a'])
        dHdy = (cpg['n'] * np.sign(cpg['y'][t,:] - y0) * (abs((cpg['y'][t,:] - y0)/cpg['b'])**(cpg['n']-1)))/ (cpg['b'])

        dHdx = dHdx / np.sqrt(dHdx*dHdx + dHdy*dHdy)
        dHdy = dHdy / np.sqrt(dHdx*dHdx + dHdy*dHdy)

        dx = dHdx * gamma * (1-H) + cpg['w'] * dHdy;
        dy = dHdy * gamma * (1-H) - cpg['w'] * dHdx;

        dz_coup = supelliptRot(cpg['a'].T, cpg['b'].T, cpg['n'], np.array([cpg['x'][t,:] - x0, cpg['y'][t,:] - y0]).T, cpg['phase_lags'])
        dx = dx + ( dz_coup[:,0].T - (cpg['x'][t,:] - x0) )/lambdaa
        
        dy = dy + ( dz_coup[:,1].T - (cpg['y'][t,:] - y0) )/lambdaa

        for value in range(6):
            truth = (cpg['CPGStanceDelta'].flatten())[value]
            if truth:
                cpg['xGr'][value] = cpg['x'][t,value]

        x_pos = abs(cpg['nomY'] - cpg['s1OffsetY'])

        dx_fact = 1 / (x_pos * np.reciprocal(np.cos(cpg['s1OffsetAngY'] + cpg['legs'][0,shoulders1]))**2)

        dx_const = -1 * cpg['scaling'] * cpg['desired_speed']* dx_fact; 
        dx_const = dx[0,:]
        truther = False
        updStab = np.logical_or(cpg['CPGStance'], (dy[0]<0))

    else:
        dx = 0;
        dy = 0;
        truther = True
        dx_const = 0;
        updStab = np.logical_or(cpg['CPGStance'], np.array([False,False,False,False,False,False]))

    #Calculate dynOffsetInc
    
    cpg['pid'].update(dt, dynOffsetError, updStab); 
    cpg['dynOffset'] = cpg['pid'].getCO();
    cpg['dynOffsetInc'] = cpg['pid'].getDeltaCO();

    # Apply dynOffsetInc
    #Integrate dx & dy to produce joint commands
    cpg['x'][t+1,:] = cpg['x'][t,:] + dx * dt + cpg['dynOffsetInc'][0,:]
    cpg['xGr'] = cpg['xGr'] + dx_const * dt + cpg['dynOffsetInc'][0,:]
    cpg['y'][t+1,:] = cpg['y'][t,:] + dy * dt + cpg['dynOffsetInc'][1,:]

    #Command CPG-generated values to joints

    yOut = cpg['y'][t+1,:]
    xOut = np.zeros([1,6])

    for value in range(6):
        truth = cpg['CPGStance'][value]
        if truth:
            xOut[:,value] = cpg['xGr'][value]
        else:
            xOut[:,value] = cpg['x'][t+1,value]
    
    cpg['legs'][0,0:18:3] = limitValue((cpg['nomOffset'][0,:] + cpg['shouldersCorr'] * xOut), pi/2 * cpg['scaling'])
    cpg['legs'][0,1:19:3] = cpg['nomOffset'][1,:] + cpg['shouldersCorr'] * np.maximum(y0, yOut)

    # JOINT 3 - FOR WALKING TRIALS

    cpg['legs'][0,0:18:3] = cpg['legs'][0,0:18:3]/cpg['scaling']
    cpg['legs'][0,1:19:3] = cpg['legs'][0,1:19:3]/cpg['scaling']


    I = (np.logical_or(cpg['CPGStance'], dy < 0));
    dist = cpg['nomY']
    indicies = []
    for index in range(6):
        if truther:
            truth = I[index]
        else:
            truth = I[0,index]
        if truth:
            indicies.append(index)

    angs = groundIK(cpg,dist,indicies)

    for index in indicies:
        cpg['legs'][0,2+index*3] = angs[2+index*3] +(cpg['dynOffset'][2,index]/cpg['scaling'])  

    
    cpg['legs'] = np.reshape(cpg['legs'][0:18],[1,18]);

    positions = cpg['xmk'].getLegPositions(cpg['legs']);

    return cpg
