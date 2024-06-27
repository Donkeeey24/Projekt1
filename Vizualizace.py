import pybullet as p
import pybullet_data
import math
import time
import os
import inspect
import Kinematika

physics_client = p.connect(p.GUI) #spusti simulaci
p.setGravity(0, 0, 0) #gravitace
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #nastavi kde se hledaji soubory - pro nacteni zakladni plochy
planeId = p.loadURDF("plane.urdf") #nacte zakladni plochu
p.setRealTimeSimulation(1) #rychlost simulace

current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
start_pos = [0, 0, 0] #zacatecni pozice X,Y,Z
start_orientation = p.getQuaternionFromEuler([0, 0, 0]) #zacatecni uhly
print(current_dir)
body_id = p.loadURDF("C:/Users/tomas/OneDrive/Plocha/diplomka/kuka_kr6/urdf/kuka_kr6_r900_mit_suction_gripper.urdf",start_pos,start_orientation, useFixedBase=True)
#nacteni roboticke ruky



'''
p.setJointMotorControlArray(
    body_id, range(6), p.POSITION_CONTROL,
    targetPositions=[0.4] * 6)
'''
joint_id_list = []
joint_pos_list = []
while(1):
    #def GetJointInfo():
    joint1 = -p.getJointState(body_id, 1)[0] #uhel natoceni jednotlivych kloubu
    joint2 = -p.getJointState(body_id, 2)[0]
    joint3 = -p.getJointState(body_id, 3)[0]
    joint4 = -p.getJointState(body_id, 4)[0]
    joint5 = -p.getJointState(body_id, 5)[0]
    joint6 = -p.getJointState(body_id, 6)[0]
        #return joint1, joint2, joint3, joint4, joint5, joint6
    #print(joint1, joint2, joint3, joint4, joint5, joint6)
    pos = Kinematika.ForwardKinematics(joint1, joint2, joint3, joint4, joint5, joint6) #dosazeni do DH
    end_pos = pos.endpoint()
    print(end_pos) #vysledek polohy pomoci DH

    joint_positions = [j[0] for j in p.getLinkStates(body_id, range(12))]
    print(joint_positions[8]) #vysledek polohy ze simulace

    time.sleep(0.1)

