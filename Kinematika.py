#dopredna kinematika - DH notace
import math as m
import numpy as np

from sympy import *

class ForwardKinematics:

    def __init__(self, th_1, th_2, th_3, th_4, th_5, th_6):
        self.a = [0.025, 0.455, 0.035, 0, 0, 0] #hodnoty a pro danou robotickou ruku (Kuka KR6 R 900)
        self.d = [0.4, 0, 0, 0.420, 0, 0.08] #hodnoty d
        self.alpha = [np.deg2rad(90), np.deg2rad(0), np.deg2rad(90), np.deg2rad(-90), np.deg2rad(90), np.deg2rad(0)] #hodnoty alpha
        self.A = []
        self.theta = (th_1, th_2, th_3 + np.deg2rad(90), th_4, th_5, th_6) #hodnoty theta

    def GetEndJoint(self):
        pos = np.eye(4, dtype=float)
        for i in range(len(self.a)):
            self.A.append([[m.cos(self.theta[i]), -m.cos(self.alpha[i]) * m.sin(self.theta[i]),
                            m.sin(self.alpha[i]) * m.sin(self.theta[i]), self.a[i] * m.cos(self.theta[i])],
                           [m.sin(self.theta[i]), m.cos(self.alpha[i]) * m.cos(self.theta[i]),
                            -m.sin(self.alpha[i]) * m.cos(self.theta[i]), self.a[i] * m.sin(self.theta[i])],
                           [0, m.sin(self.alpha[i]), m.cos(self.alpha[i]), self.d[i]],
                           [0, 0, 0, 1]]) #DH matice
        return self.A

    def endpoint(self):
        A = np.eye(4)
        A = self.GetEndJoint()
        pos = np.eye(4)
        for i in range(6):
            pos = np.matmul(pos, A[i])
        return pos #pronasobeni matic

    def endpointfor3(self):
        A = np.eye(4)
        A = self.GetEndJoint()
        pos = np.eye(4)
        for i in range(4):
            pos = np.matmul(pos, A[i])
        return pos


class ForwardKinematicssym: #dopredna kinematika obecne

    def __init__(self, th_1, th_2, th_3, th_4, th_5, th_6):
        self.a = [0.025, 0.455, 0.035, 0, 0, 0]
        self.d = [0.4, 0, 0, 0.420, 0, 0.08]
        self.alpha = [np.deg2rad(90), np.deg2rad(0), np.deg2rad(90), np.deg2rad(-90), np.deg2rad(90), np.deg2rad(0)]
        self.A = []
        self.theta = (th_1, th_2, th_3 + np.deg2rad(90), th_4, th_5, th_6)

    def GetEndJoint(self):
        pos = np.eye(4, dtype=float)
        for i in range(len(self.a)):
            self.A.append([[cos(self.theta[i]), -cos(self.alpha[i]) * sin(self.theta[i]),
                            sin(self.alpha[i]) * sin(self.theta[i]), self.a[i] * cos(self.theta[i])],
                           [sin(self.theta[i]), cos(self.alpha[i]) * cos(self.theta[i]),
                            -sin(self.alpha[i]) * cos(self.theta[i]), self.a[i] * sin(self.theta[i])],
                           [0, sin(self.alpha[i]), cos(self.alpha[i]), self.d[i]],
                           [0, 0, 0, 1]])
        return self.A

    def Aimat(self, i):
        A = np.eye(4)
        A = self.GetEndJoint()
        return A[i]


class InverseKinematics: #inverzni kinematika
    def __init__(self, matrix):

        self.matrix = matrix


    def first3positions(self): #vypocet prvnich 3 kloubu
        enjoint = np.eye(4, dtype=np.float64)
        enjoint [2,3] = -0.08 #rozmer koncoveho clenu

        newmat = np.matmul(enjoint, matrix) #ziskani koncove pozice bez koncoveho clenu

        xc = newmat[0,3]
        yc = newmat[1,3]
        zc = newmat[2,3]


        theta1 = m.atan2(yc, xc) #uhel 1 dan pozici yc,xc
        xcnew = xc * m.cos(-theta1)-yc * m.sin(-theta1) #matematicke posunuti uhlu 1 do 0
        ycnew = yc * m.cos(-theta1)-xc * m.sin(-theta1)

        L1 = abs(xcnew-0.025) #dal geometricke dopocitani uhlu 2,3
        L4 = abs(zc-0.4)
        L2 = m.sqrt(L1**2+L4**2)
        L3 = m.sqrt(0.420**2+0.035**2)

        if (0.455**2+L2**2-L3**2)/(2*0.455*L2) > 1:
            thetaC = m.acos(1)
        elif (0.455**2+L2**2-L3**2)/(2*0.455*L2) < -1:
            thetaC = m.acos(-1)
        else:
            thetaC = m.acos((0.455**2+L2**2-L3**2)/(2*0.455*L2))

        if (0.455**2+L3**2-L2**2)/(2*0.455*L3) > 1:
            thetaD = m.acos(1)
        elif (0.455**2+L3**2-L2**2)/(2*0.455*L3) < -1:
            thetaD = m.acos(-1)
        else:
            thetaD = m.acos((0.455**2+L3**2-L2**2)/(2*0.455*L3))

        thetaB = m.atan2(L4,L1)
        thetaE = m.atan2(0.035,0.42)
        print(thetaB)
        print(thetaC)

        if xcnew > 0.025:
            if L4 > 0:
                theta2 = thetaB - thetaC
            else:
                theta2 = thetaB - thetaC + m.deg2rad(180)
        else:
            theta2 = -(thetaB + thetaC)

        theta3 = -(thetaD+thetaE)+np.deg2rad(180)


        FWD3 = ForwardKinematics(theta1, theta2, theta3, np.deg2rad(0), np.deg2rad(0), np.deg2rad(0))
        pos = FWD3.endpoint()


        Tpos = np.transpose(pos[0:3, 0:3]) #dal dopocitani uhlu 4,5,6
        wristorientation = np.matmul(self.matrix[0:3,0:3], Tpos)
        if m.atan2(wristorientation[2, 2], m.sqrt(1-wristorientation[2, 2]**2))>0:
            theta5 = m.atan2(wristorientation[2, 2], m.sqrt(1 - wristorientation[2, 2] ** 2))
        else:
            theta5 = m.atan2(wristorientation[2, 2], -m.sqrt(1 - wristorientation[2, 2] ** 2))

        if theta5>0:
            theta4 = m.atan2(wristorientation[0, 2], -wristorientation[1, 2])
        else:
            theta4 = -m.atan2(-wristorientation[0, 2], wristorientation[1, 2])

        if theta5>0:
            theta6 = m.atan2(-wristorientation[2, 0], wristorientation[2, 1])
        else:
            theta6 = m.atan2(wristorientation[2, 0], -wristorientation[2, 1])

        return (np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3), np.rad2deg(theta4)+90, np.rad2deg(theta5)-90, np.rad2deg(theta6)+90)


q1 = symbols("q1")
q2 = symbols("q2")
q3 = symbols("q3")
q4 = symbols("q4")
q5 = symbols("q5")
q6 = symbols("q6")

mat = ForwardKinematicssym(q1, q2, q3, q4, q5, q6)


def getAi():
    A = []
    for i in range(6):
        A.append(mat.Aimat(i))
    return A

def calculate_matrix(x, y, z, alpha, beta, gamma): #vytvoreni matice polohy ze zadanych souradnic
    matrix = np.eye(4, dtype=np.float64)
    alpha = np.deg2rad(alpha)
    beta = np.deg2rad(beta)
    gamma = np.deg2rad(gamma)
    ca = cos(alpha)
    cb = cos(beta)
    cy = cos(gamma)
    sa = sin(alpha)
    sb = sin(beta)
    sy = sin(gamma)
    matrix[0, 0] = cb * cy
    matrix[0, 1] = cy * sb * sa - sy * ca
    matrix[0, 2] = cy * sb * ca + sy * sa

    matrix[1, 0] = sy * cb
    matrix[1, 1] = sy * sb * sa + cy * ca
    matrix[1, 2] = sy * sb * ca - cy * sa

    matrix[2, 0] = -sb
    matrix[2, 1] = cb * sa
    matrix[2, 2] = cb * ca

    matrix[0, 3] = x
    matrix[1, 3] = y
    matrix[2, 3] = z


    return matrix


test = getAi()
#souradnice X,Y,Z rotace okolo X,Y,Z
x = 0.98
y = 0
z = 0.435
R_x = 45
R_y = -90
R_z = 135
matrix = calculate_matrix(x, y, z, R_x, R_y, R_z)
print(matrix)
t2 = InverseKinematics(matrix)
print(t2.first3positions())
#print('vysledek geometricke inverzni kinematiky')
#fwdkin = ForwardKinematics(np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0))
#fwdinv = ForwardKinematics(np.deg2rad(0), np.deg2rad(2.68), np.deg2rad(-4.764), np.deg2rad(0), np.deg2rad(87.93), np.deg2rad(-90))
#print('vysledek DH')
#print(fwdkin.endpoint())
#print(fwdinv.endpoint())
