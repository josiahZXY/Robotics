#! /usr/bin/python3
import hebi 
import numpy as np
import sys
import tty
import termios
import time 
import copy
import setup
import rospy
import sys, select, termios, tty
from Tools.transforms import *
from Tools.PIDController import PIDController
from CPG.updateCPGStance import *
# from CPG.laptopCPG import *
# from CPG.noFeedCPG import *
from CPG.newCPG import *
import rospkg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import String
import math
from matplotlib import pyplot as plt
global new_direction
new_direction='0'


jointPositions = np.zeros((1,18))
def joint_callback(data):
    pos = np.array(data.position)
    pos = pos.reshape(1,18)
    pos = pos.reshape(3,6)
    pos = pos.T
    pos = pos.reshape(1,18)
    jointPositions = pos


def callback(data):
    global new_direction
    #rospy.loginfo(rospy.get_caller_id()+"Heard %s",data.data)
    new_direction=data.data




#Setup Modules and Kinematics Object
from setup.xMonsterKinematics import *
xmk = HexapodKinematics();


#initialize the complementary filter object
rospack = rospkg.RosPack()
pkgPath = rospack.get_path('xMonsterCPG')
offsets = np.load(pkgPath+'/src/xMonsterCPG/setup/setupFiles/offsets.npy',allow_pickle=True, encoding='latin1')
rospy.init_node('laptop_cpg')



T_elaspsed = 0
T = 5000 #Time in seconds code operates for
nIter = int(round(T/0.01)) 

#creating cpg dict
cpg = {
    'initLength': 0,
    's':np.array([0.30, 0.30, 0.30, 0.30, 0.30, 0.30]),
    # 'h': 0.23,
    # 'nomX': np.array([0.40, 0.40, 0.0, 0.0, -0.40, -0.40]),
    # 'nomY': np.array([0.38, -0.38, 0.50,  -0.50, 0.38, -0.38]),

    'h': 0.2249,
    'nomX': np.array([0.51611, 0.51611, 0.0575, 0.0575, -0.45861, -0.45861]),
    'nomY': np.array([0.23158, -0.23158, 0.51276, -0.51276,  0.33118, -0.33118]),
    's1OffsetY': np.array([0.2375*sin(pi/6), -0.2375*sin(pi/6), 0.1875, -0.1875, 0.2375*sin(pi/6), -0.2375*sin(pi/6)]),#robot measurement;  distance on x axis from robot center to
    's1OffsetAngY': np.array([-pi/3, pi/3, 0, 0, pi/3, -pi/3]),
    'n': 2, #limit cycle shape 2:standard, 4:super  
    'b': 0.5*np.ones(6)*2, #np.array([.4, .4, .4, .4, .4, .4]), #TUNEABLE: step height in radians %1.0
    'scaling': 10, #TUNEABLE: shifts the units into a reasonable range for cpg processing (to avoid numerical issues)
    'shouldersCorr': np.array([-1, 1, -1, 1, -1, 1]),
    'phase_lags': np.array([pi, pi, 0, pi, 0, pi]),
    'dynOffset': np.zeros([3,6]), #Offset on joints developed through constraining 
    'dynOffsetInc': np.zeros([3,6]), #Increment since last iteration
    'x': np.zeros([nIter,6]), #TUNEABLE: Initial CPG x-positions
    'y': np.zeros([nIter,6]), #TUNEABLE: Initial CPG y-positions
    'x0': np.zeros([1,6]), #limit cycle center x
    'y0': np.zeros([6,6]), #limit cycle center y
    'legs': np.zeros([1,18]), #Joint angle values
    'elbowsLast': np.zeros([1,6]), #elbow values
    'torques': np.zeros([1,18]), #Joint torque values
    'torqueOffsets': offsets[2], #Joint torque offset values
    'gravCompTorques': np.zeros([1,18]), #Joint torque values
    'forces': np.zeros([3,6]), #ee force values
    'gravCompForces': np.zeros([3,6]), #Joint torque values
    'forceStance': np.zeros([1,6]), #grounded legs determined by force
    'CPGStance': np.array([False,False,False,False,False,False]), #grounded legs determined by position (lower tripod)
    'CPGStanceDelta': np.zeros([1,6]), #grounded legs determined by position (lower tripod)
    'CPGStanceBiased': np.zeros([1,6]), #grounded legs determined by position (lower tripod)
    'comm_alpha': 1.0, #commanded alpha in the complementary filter (1-this) is the measured joint angles
    'move': True, #true: walks according to cpg.direction, false: stands in place (will continue to stabilize); leave to true for CPG convergence
    'xmk': xmk, #Snake Monster Kinematics object
    'pose': np.eye(3), #%SO(3) describing ground frame w.r.t world frame
    'R': SE3(np.eye(3),[0, 0, 0]), #SE(3) describing body correction in the ground frame
    'G': np.eye(3), #SO(3) describing ground frame w.r.t world frame
    'tp': np.zeros([4,1]),
    'dynY': 0, 
    'vY': 0,
    'direction' : 'backwards',
    'fullStepLength' : 20000,
    't' : 0
}
#def getKey():
#    tty.setraw(sys.stdin.fileno())
#    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
#    if rlist:
#        key = sys.stdin.read(1)
#    else:
#        key = ''
#
#    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#    return key
#

def readchar():
	fd = sys.stdin.fileno()
	old_settings = termios.tcgetattr(fd)
	try:
		tty.setraw(sys.stdin.fileno())
		ch = sys.stdin.read(1)
	finally:
		termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
	return ch
 
def readkey(getchar_fn=None):
	getchar = getchar_fn or readchar
	c1 = getchar()
	if ord(c1) != 0x1b:
		return c1
	c2 = getchar()
	if ord(c2) != 0x5b:
		return c1
	c3 = getchar()
	return chr(0x10 + ord(c3) - 65)

def command_callback(data):
    cpg['direction'] = data.data
    cpg['t'] = cpg['initLength'] + 1
    print(cpg['direction'])


#cpg['pid'] = PIDController([3,6],0.,1,0.005,0.01, pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling']); # BABY GAINS for climbing
cpg['pid'] = PIDController([3,6],0.000,2,0.005,0.000,pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling']);
#cpg['torquepid'] = PIDController([1,6],0.000,0.000,0.000,0.000,pi/2 *cpg['scaling'], pi/2 * cpg['scaling'], pi/2 * cpg['scaling'], pi/2*cpg['scaling']);
cpg['T']=SE3(np.eye(3),np.array([0, 0,cpg['h']]))#SE(3) describing desired body orientation in the ground frame


# FIXED GROUND ASSUMPTION
GG = rotx(0/180*pi); # R: This thing is supposed to store the orientation of the ground
[Tx,_,_] = eulerSO3(GG); 
actual_bh = 0.25; 
Xoffset = 0.08; #based on fixed ground  R: This is essentialy to push the robot forward in y, based on the inclination
desired_transY = -(actual_bh * np.tan(Tx) + Xoffset); # R: Compute the desired "forward push" in y
cpg['eePos'] = np.vstack((cpg['nomX'],cpg['nomY'], -cpg['h'] * np.ones([1,6]) )) # R: Compute the EE positions in body frame
# print(cpg['eePos'])
# [[ 0.4   0.4   0.    0.   -0.4  -0.4 ]
#  [ 0.38 -0.38  0.5  -0.5   0.38 -0.38]
#  [-0.23 -0.23 -0.23 -0.23 -0.23 -0.23]]

ang = cpg['xmk'].getLegIK(cpg['eePos']); #R: This gives the angles corresponding to each of the joints



cpg['nomOffset'] = np.reshape(ang[0:18],[6,3]).T; # R: These are the angles corresponding to the rest position

# a = 0.16*np.ones([1,6])
a = np.array([[0.4092], [0.4092], [0.55], [0.55], [0.4092], [0.4092]])/ math.sqrt(3)
# a = np.array([[0.65], [0.65], [0.4092], [0.4092], [0.55], [0.55]])/math.sqrt(2)
a = np.reshape(a, (1, 6))
aMin = -10;
a[a < aMin] = aMin;
cpg['a'] =   a   #* cpg['scaling']

cpg['b'] = cpg['b'] #* cpg['scaling']
cpg['nomOffset'] = cpg['nomOffset'] * cpg['scaling']

#CPG Initialization
cpg['wStance'] = 0.80*6; #cpg anglular speed [rad]/[sec]
cpg['wSwing'] = cpg['wStance'] * 6.0;
cpg['K'] = np.array( [[0, -1, -1,  1,  1, -1],
                     [-1,  0,  1, -1, -1,  1], 	 
                     [-1,  1,  0, -1, -1,  1],
                     [ 1, -1, -1,  0,  1, -1],
                     [ 1, -1, -1,  1,  0, -1],
                     [-1,  1,  1, -1, -1,  0]])

u = np.array([-pi/5, 7*pi/5 ,6*pi/5 ,9*pi/5 ,8*pi/5 ,7*pi/5])

cpg['x'][0,:] =  cpg['a'] * np.array([1,-1,-1,1,1,-1]); # R: Initialize the x and y values of the cpg cycle
cpg['y'][0,:] =  np.zeros(6) #np.array([-0.7216 ,   0.7216 ,   0.7216,   -0.7216,   -0.7216,    0.7216]);
cpg['xGr'] = cpg['x'][0,:] # R: x or first joint angle values, corresponding to grounded legs

stance_duration = (cpg['a'] + cpg['b'])/(cpg['wStance'])
cpg['desired_speed'] = cpg['s'] / (stance_duration[0,:])

cpg['cx'] = np.array([0, 0, 0, 0, 0, 0]) * math.pi
cpg['cy'] = np.array([0, 0, 0, 0, 0, 0]) * math.pi
#done initializing cpg

prevFeedbackTime = time.time();
rate = rospy.Rate(100.0)



rospy.Subscriber("m6/joint_states", JointState, joint_callback)
rospy.Subscriber("/hexapod/direction/command", String, command_callback)

rospy.Subscriber("further_direction", String, callback, queue_size=1)


    

pub_base1 = rospy.Publisher("/m6/base1_position_controller/command", Float64)
pub_base2 = rospy.Publisher("/m6/base2_position_controller/command", Float64)
pub_base3 = rospy.Publisher("/m6/base3_position_controller/command", Float64)
pub_base4 = rospy.Publisher("/m6/base4_position_controller/command", Float64)
pub_base5 = rospy.Publisher("/m6/base5_position_controller/command", Float64)
pub_base6 = rospy.Publisher("/m6/base6_position_controller/command", Float64)

pub_shoulder1 = rospy.Publisher("/m6/shoulder1_position_controller/command", Float64)
pub_shoulder2 = rospy.Publisher("/m6/shoulder2_position_controller/command", Float64)
pub_shoulder3 = rospy.Publisher("/m6/shoulder3_position_controller/command", Float64)
pub_shoulder4 = rospy.Publisher("/m6/shoulder4_position_controller/command", Float64)
pub_shoulder5 = rospy.Publisher("/m6/shoulder5_position_controller/command", Float64)
pub_shoulder6 = rospy.Publisher("/m6/shoulder6_position_controller/command", Float64)

pub_elbow1 = rospy.Publisher("/m6/elbow1_position_controller/command", Float64)
pub_elbow2 = rospy.Publisher("/m6/elbow2_position_controller/command", Float64)
pub_elbow3 = rospy.Publisher("/m6/elbow3_position_controller/command", Float64)
pub_elbow4 = rospy.Publisher("/m6/elbow4_position_controller/command", Float64)
pub_elbow5 = rospy.Publisher("/m6/elbow5_position_controller/command", Float64)
pub_elbow6 = rospy.Publisher("/m6/elbow6_position_controller/command", Float64)


pub_control_state = rospy.Publisher("/m6/control_state", Float64)

control_state = 0.0
while not rospy.is_shutdown():

 
    #Time feedback
    timeNow = time.time()
    dt = max(min(timeNow - prevFeedbackTime, 0.01),0.01); #Ensure 25Hz-100Hz for CPG stability
    prevFeedbackTime = timeNow



    #Updating a value
    # a = 0.16*np.ones([1, 6])
    a = np.array([[0.2618],[0.2618],[0.4618],[0.4618],[0.2618],[0.2618]])/math.sqrt(3)
    # a = np.array([[0.65], [0.65], [0.4092], [0.4092], [0.55], [0.55]])/ math.sqrt(2)
    a = np.reshape(a, (1, 6))
    aMin = -10
    a[a < aMin] = aMin
    
    # Moving Right
    # if(new_direction=='1' or new_direction=='2'):
    #     if(new_direction=='1'):
    #         print('right')
    #     elif(new_direction=='2'):
    #         print('turn')
    #     a[0,0]=a[0,0]*1.4
    #     a[0,2]=a[0,2]*1.4
    #     a[0,4]=a[0,4]*1.4
    #     a[0,1]=a[0,1]*0.6
    #     a[0,3]=a[0,3]*0.6
    #     a[0,5]=a[0,5]*0.6
    # #Moving left
    # elif(new_direction=='-1'):
    #     print('left')
    #     a[0,0]=a[0,0]*0.6
    #     a[0,2]=a[0,2]*0.6
    #     a[0,4]=a[0,4]*0.6
    #     a[0,1]=a[0,1]*1.4
    #     a[0,3]=a[0,3]*1.4
    #     a[0,5]=a[0,5]*1.4
    # else:
    #     print('straight')

    import numpy as np
    import torch
    import torch.nn as nn


    class GCN(nn.Module):
        def __init__(self, in_dim, out_dim, dropout=0.):
            super(GCN, self).__init__()
            self.trans_msg = nn.Linear(in_dim, out_dim)
            self.nonlinear = nn.ReLU()  # less than 0 = 0
            self.dropout = nn.Dropout(dropout)

        def forward(self, x: torch.Tensor, m: torch.Tensor):  # x : nodes feature matrix   m : adjacent matrix
            x_msg = self.trans_msg(x)
            x_msg = self.nonlinear(x_msg)
            x_msg = self.dropout(x_msg)

            # print('x')
            # print(x)
            # print(m)
            row_degree = torch.sum(m, dim=1, keepdim=True)  # (N, 1)
            col_degree = torch.sum(m, dim=0, keepdim=True)  # (1, N)
            degree = torch.mm(torch.sqrt(row_degree), torch.sqrt(col_degree))  # (N, N)
            out = torch.mm(m / degree, x_msg)

            return out


    class Net(nn.Module):
        def __init__(self, nodes_num, embedding_dim, hidden_dims, hidden_1, hidden_2, hidden_3, num_classes,
                     dropout=0.):
            super(Net, self).__init__()

            self.node_embedding = nn.Embedding(nodes_num, embedding_dim)
            gcns = []
            in_dim = embedding_dim
            for d in hidden_dims:
                gcns.append(GCN(in_dim, d, dropout))
                in_dim = d
            self.gcns = nn.ModuleList(gcns)

            self.fclayer_1 = nn.Sequential(nn.Linear(in_dim, hidden_1), nn.ReLU(True))
            self.fclayer_2 = nn.Sequential(nn.Linear(hidden_1, hidden_2), nn.ReLU(True))
            self.fclayer_3 = nn.Sequential(nn.Linear(hidden_2, hidden_3), nn.ReLU(True))
            self.fclayer_4 = nn.Sequential(nn.Linear(hidden_3, num_classes))


        def gcn_maxpooling(self, x, bm):
            batch_size = torch.max(bm) + 1
            out = []
            for i in range(batch_size):
                inds = (bm == i).nonzero()[:, 0]
                x_ind = torch.index_select(x, dim=0, index=inds)
                out.append(torch.max(x_ind, dim=0, keepdim=False)[0])
            out = torch.stack(out, dim=0)

            return out

        def gcn_meanpooling(self, x, lens):
            batch_size = torch.max(bm) + 1
            out = []
            for i in range(batch_size):
                inds = (bm == i).nonzero()[:, 0]
                x_ind = torch.index_select(x, dim=0, index=inds)
                out.append(torch.mean(x_ind, dim=0, keepdim=False))
            out = torch.stack(out, dim=0)

            return out

        def gcn_sumpooling(self, x, lens):
            batch_size = torch.max(bm) + 1
            out = []
            for i in range(batch_size):
                inds = (bm == i).nonzero()[:, 0]
                x_ind = torch.index_select(x, dim=0, index=inds)
                out.append(torch.sum(x_ind, dim=0, keepdim=False))
            out = torch.stack(out, dim=0)

            return out


        def forward(self, x, m, bm):
            # x_emb = self.node_embedding(x)  # (N, embedding_dim)
            out = x
            for ml in self.gcns:
                out = ml(out, m)
            output = self.gcn_meanpooling(out, bm)  # (batch_size, out_dim)

            logits = self.fclayer_1(output)  # (batch_size, num_classes)
            logits = self.fclayer_2(logits)
            logits = self.fclayer_3(logits)
            logits = self.fclayer_4(logits)

            return logits


    def edges_to_matrix(node_num, edges):
        m = np.zeros(shape=(node_num, node_num), dtype=np.uint8)
        m[edges[:, 0], edges[:, 1]] = 1
        m[edges[:, 1], edges[:, 0]] = 1
        m[np.arange(node_num), np.arange(node_num)] = 1

        return m

    def GNN_model(a, a_0, camera):

        edges = np.array([[0, 6], [1, 6], [2, 6], [3, 6], [4, 6], [5, 6]])
        m = edges_to_matrix(7, edges)

        n = torch.tensor([[-1.0000, a[0], -0.5236, a_0[0]],
                          [1.0000, a[1], -0.5236, a_0[1]],
                          [-1.0000, a[2], 0.0000, a_0[2]],
                          [1.0000, a[3], 0.0000, a_0[3]],
                          [-1.0000, a[4], 0.5236, a_0[4]],
                          [1.0000, a[5], 0.5236, a_0[5]],
                          [camera, camera, camera, camera]]).float()

        # n = torch.tensor([[-1.0000, a[0], -0.5236, a_0[0], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #                   [1.0000, a[1], -0.5236, a_0[1], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #                   [-1.0000, a[2], 0.0000, a_0[2], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #                   [1.0000, a[3], 0.0000, a_0[3], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #                   [-1.0000, a[4], 0.5236, a_0[4], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #                   [1.0000, a[5], 0.5236, a_0[5], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #                   camera]).float()


                          # [camera[0],camera[1],camera[2],camera[3],camera[4],camera[5],camera[6],camera[7],
                          #  camera[8],camera[9],camera[10],camera[11],camera[12],camera[13],camera[14],camera[15]]]).float()

        m = torch.from_numpy(m).float()

        bm = torch.tensor([0, 0, 0, 0, 0, 0, 0], dtype=torch.int32)
        out = model(n, m, bm)
        return out


    model = torch.load('/home/zxy/yuna_ws/src/Yuna-IMU-CPG-python-CNN_2_GNN/src/xMonsterCPG/model/model_ros_0415.pkl')
    a_exist = np.array([0.16, 0.16, 0.16, 0.16, 0.16, 0.16])
    a_0 = np.array([0.16, 0.16, 0.16, 0.16, 0.16, 0.16])
    # sensor_nodes = new_direction.split()
    # sensor_nodes = map(float, sensor_nodes)
    # sensor_nodes = list(sensor_nodes)
    #
    # if len(sensor_nodes) == 32:
    #     sensor_nodes = np.array(sensor_nodes)
    # else:
    #     sensor_nodes = np.array([0.19102599, -0.52592915, -0.40611798, 0.44481376, -0.51975244, -0.38592952, 4.4749866, -0.2079358,
    #                              -0.62693805, -0.7610763, -0.718664, 0.3539446, 1.5845952, 0.7998093, -0.51186585, -0.4684513,
    #                              -1.0478846, -1.1007576, 0.39981452, -0.070312545, 0.49643987, 2.3545716, 0.23529112, 1.2310095,
    #                              -1.0362737, -0.5698456, -0.7145635, -1.4313295, -0.58287436, -0.14168167, 0.16352348, -0.9016383])
        # sensor_nodes = np.array([0.3862853, -0.48032784, -1.1083046, 4.415415, -1.1139008, -0.6044526, 1.8194215, -0.66718507,
        #                          -1.5092801, 0.48015743, 0.8925237, 0.9299811, -1.1219298, -0.68828607, -1.1677648, -0.46235186])



    camera = int(new_direction)

    if camera == 2:
        camera = -1
        print('left')
    if camera == 3:
        print('no object')
        camera = 1
    if camera == 0:
        print('forward')
    if camera == 1:
        print('right')


    #
    bm = torch.tensor([0, 0, 0, 0, 0, 0, 0], dtype=torch.int32)
    b = GNN_model(a_exist, a_0, camera)
    new_a=b.detach().numpy()
    # print(new_a)
    # time.sleep(0.2)
    # print(a)
    # print(new_direction)
    # print(a)
    cpg['a'] =   a  # * cpg['scaling']
    # print(cpg['a'])

    #Position feedback
    cpg['legs'] = cpg['comm_alpha'] * cpg['legs'] + (1-cpg['comm_alpha']) * jointPositions # R: Basically weighing the cpg computation and the position feedback


    cpg,positions  = CPG(cpg,cpg['t'], dt)
    t=cpg['t']
    # print(positions)

    # plt.subplot(231)
    # plt.plot(cpg['x'][t+1,:][0],cpg['y'][t+1,:][0],".b")
    # plt.subplot(234)
    # plt.plot(cpg['x'][t+1,:][1],cpg['y'][t+1,:][1],".b")
    # plt.subplot(232)
    # plt.plot(cpg['x'][t+1,:][2],cpg['y'][t+1,:][2],".b")
    # plt.subplot(235)
    # plt.plot(cpg['x'][t+1,:][3],cpg['y'][t+1,:][3],".b")
    # plt.subplot(233)
    # plt.plot(cpg['x'][t+1,:][4],cpg['y'][t+1,:][4],".b")
    # plt.subplot(236)
    # plt.plot(cpg['x'][t+1,:][5],cpg['y'][t+1,:][5],".b")
    # plt.pause(0.01)
    # plt.show()

    if cpg['t'] > cpg['initLength']:
        control_state = 1.0


    if cpg['t'] > (cpg['initLength'] + 500 ) and cpg['t'] < cpg['fullStepLength']:
        cpg['move'] = True
        commanded_position = cpg['legs'][0,:]
        # print(cpg['legs'][0,:])
        # commanded_position = np.zeros(18)

        pos_leg0 = positions[:, 0]
        pos_leg1 = positions[:, 1]
        pos_leg2 = positions[:, 2]
        pos_leg3 = positions[:, 3]
        pos_leg4 = positions[:, 4]
        pos_leg5 = positions[:, 5]

        with open('pos_leg0.txt', 'a') as f:
            f.write(str(pos_leg0[0]) + ',' + str(pos_leg0[1]) + ',' + str(pos_leg0[2]) + '\n')
        with open('pos_leg1.txt', 'a') as f:
            f.write(str(pos_leg1[0]) + ',' + str(pos_leg1[1]) + ',' + str(pos_leg1[2]) + '\n')
        with open('pos_leg2.txt', 'a') as f:
            f.write(str(pos_leg2[0]) + ',' + str(pos_leg2[1]) + ',' + str(pos_leg2[2]) + '\n')
        with open('pos_leg3.txt', 'a') as f:
            f.write(str(pos_leg3[0]) + ',' + str(pos_leg3[1]) + ',' + str(pos_leg3[2]) + '\n')
        with open('pos_leg4.txt', 'a') as f:
            f.write(str(pos_leg4[0]) + ',' + str(pos_leg4[1]) + ',' + str(pos_leg4[2]) + '\n')
        with open('pos_leg5.txt', 'a') as f:
            f.write(str(pos_leg5[0]) + ',' + str(pos_leg5[1]) + ',' + str(pos_leg5[2]) + '\n')
        print('saving txt..........')


    else:
        commanded_position = ang[0:18]
        #print('stopped')

    # hexapod.send_command(group_command);
    pub_control_state.publish(control_state)


    pub_base1.publish(commanded_position[0])
    pub_base2.publish(commanded_position[3])
    pub_base3.publish(commanded_position[6])
    pub_base4.publish(commanded_position[9])
    pub_base5.publish(commanded_position[12])
    pub_base6.publish(commanded_position[15])

    pub_shoulder1.publish(commanded_position[1])
    pub_shoulder2.publish(commanded_position[4])
    pub_shoulder3.publish(commanded_position[7])
    pub_shoulder4.publish(commanded_position[10])
    pub_shoulder5.publish(commanded_position[13])
    pub_shoulder6.publish(commanded_position[16])

    pub_elbow1.publish(commanded_position[2])
    pub_elbow2.publish(commanded_position[5])
    pub_elbow3.publish(commanded_position[8])
    pub_elbow4.publish(commanded_position[11])
    pub_elbow5.publish(commanded_position[14])
    pub_elbow6.publish(commanded_position[17])


    rate.sleep()

    cpg['t'] += 1
    key=readkey()
    if key=='w':
        cpg.update({'direction':'forward'})
    if key=='a':
        cpg.update({'direction':'left'})
    if key=='s':
        cpg.update({'direction':'backwards'})
    if key=='d':
        cpg.update({'direction':'right'})

   

