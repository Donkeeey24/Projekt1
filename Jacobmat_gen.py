import dill
import sympy as sym
import numpy as np
from numpy import pi
from sympy import sin, cos, Matrix, lambdify


th1, th2, th3, th4, th5, th6 = sym.symbols('th1, th2, th3, th4, th5, th6')

class ForwardKinematicssym:

    def __init__(self):

        self.a = [0.025, 0.455, 0.035, 0, 0, 0]
        self.d = [0.4, 0, 0, 0.420, 0, 0.08]
        self.alpha = [np.deg2rad(90), np.deg2rad(0), np.deg2rad(90), np.deg2rad(-90), np.deg2rad(90), np.deg2rad(0)]
        self.A = []
        self.theta = (th1, th2, th3 + np.deg2rad(90), th4, th5, th6)

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

mat = ForwardKinematicssym()


def getAi():
    A = []
    for i in range(6):
        A.append(mat.Aimat(i))
    return np.array(A)

Amat=getAi()

T2 = Amat[0] @ Amat[1]
T3 = Amat[0] @ Amat[1] @ Amat[2]
T4 = Amat[0] @ Amat[1] @ Amat[2] @ Amat[3]
T5 = Amat[0] @ Amat[1] @ Amat[2] @ Amat[3] @ Amat[4]
T6 = Amat[0] @ Amat[1] @ Amat[2] @ Amat[3] @ Amat[4] @ Amat[5]

z0= np.array([0,0,1])
z1 = Amat[1][:3,2]
z2 = T2[:3,2]
z3 = T3[:3,2]
z4 = T4[:3,2]
z5 = T5[:3,2]

p0 = np.array([0,0,0])
p1 = Amat[1][:3,3]
p2 = T2[:3,3]
p3 = T3[:3,3]
p4 = T4[:3,3]
p5 = T5[:3,3]

P = T6[:3,3]

J = Matrix([
    np.append(np.cross(z0, P - p0), z0),
    np.append(np.cross(z1, P - p1), z1),
    np.append(np.cross(z2, P - p2), z2),
    np.append(np.cross(z3, P - p3), z3),
    np.append(np.cross(z4, P - p4), z4),
    np.append(np.cross(z5, P - p5), z5),
]).T

f = lambdify('th1, th2, th3, th4, th5, th6', J, 'numpy')

dill.settings['recurse'] = True
dill.dump(f, open("jacobian", "wb"))