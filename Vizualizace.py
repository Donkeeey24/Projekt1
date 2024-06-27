import pybullet as p
import pybullet_data
import math
import time
import os
import inspect
import Kinematika

physics_client = p.connect(p.GUI)
p.setGravity(0, 0, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setRealTimeSimulation(1)

current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
start_pos = [0, 0, 0]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
print(current_dir)
body_id = p.loadURDF("C:/Users/tomas/OneDrive/Plocha/diplomka/kuka_kr6/urdf/kuka_kr6_r900_mit_suction_gripper.urdf",start_pos,start_orientation, useFixedBase=True)




'''
p.setJointMotorControlArray(
    body_id, range(6), p.POSITION_CONTROL,
    targetPositions=[0.4] * 6)
'''
joint_id_list = []
joint_pos_list = []
while(1):
    #def GetJointInfo():
    joint1 = -p.getJointState(body_id, 1)[0]
    joint2 = -p.getJointState(body_id, 2)[0]
    joint3 = -p.getJointState(body_id, 3)[0]
    joint4 = -p.getJointState(body_id, 4)[0]
    joint5 = -p.getJointState(body_id, 5)[0]
    joint6 = -p.getJointState(body_id, 6)[0]
        #return joint1, joint2, joint3, joint4, joint5, joint6
    #print(joint1, joint2, joint3, joint4, joint5, joint6)
    pos = Kinematika.ForwardKinematics(joint1, joint2, joint3, joint4, joint5, joint6)
    end_pos = pos.endpoint()
    print(end_pos)

    joint_positions = [j[0] for j in p.getLinkStates(body_id, range(12))]
    print(joint_positions[8])

    time.sleep(0.1)

