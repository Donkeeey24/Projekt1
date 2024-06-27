import numpy as np
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
import matplotlib.pyplot as plt


def DH(theta, alpha, r, d):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca, st * sa, r * ct],
        [st, ct * ca, -ct * sa, r * st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])


numA = [20, 33, 50, 70, 100, 150, 250]
eee = np.zeros(7)

for i_n in range(7):
    numnodes = numA[i_n]

    t1 = np.arange(-1, 1.1, 0.1)
    t2 = np.arange(0, 1.2, 0.1)
    t3 = np.arange(0, -1.2, -0.1)

    t = np.zeros((21 * 12 * 12, 3))
    xy = np.zeros((21 * 12 * 12, 3))

    index = 0
    for i in range(21):
        for j in range(12):
            for k in range(12):
                a = DH(t1[i], np.pi / 2, 0, 0)
                b = np.dot(a, DH(t2[j], 0, 10, 0))
                c = np.dot(b, DH(t3[k], 0, 10, 0))

                t[index] = [t1[i], t2[j], t3[k]]
                xy[index] = [c[0, 3], c[1, 3], c[2, 3]]
                index += 1

    x = xy[:, 0]
    y = xy[:, 1]

    plt.figure(1)
    plt.plot(x, y, 'r+')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('XY plocha')

    x1 = np.zeros(12 * 12)
    y1 = np.zeros(12 * 12)
    z1 = np.zeros(12 * 12)

    index = 0
    for j in range(12):
        for k in range(12):
            a = DH(0, np.pi / 2, 0, 0)
            b = np.dot(a, DH(t2[j], 0, 10, 0))
            c = np.dot(b, DH(t3[k], 0, 10, 0))

            x1[index] = c[0, 3]
            y1[index] = c[1, 3]
            z1[index] = c[2, 3]
            index += 1

    plt.figure(2)
    plt.plot(x1, z1, 'g*')
    plt.xlabel('X')
    plt.ylabel('Z')
    plt.title('XZ plocha')

    x1 = np.zeros(21 * 12)
    y1 = np.zeros(21 * 12)
    z1 = np.zeros(21 * 12)

    index = 0
    for i in range(21):
        for j in range(12):
            a = DH(t1[i], np.pi / 2, 0, 0)
            b = np.dot(a, DH(t2[j] / 2, 0, 10, 0))
            c = np.dot(b, DH(t3[j], 0, 10, 0))

            x1[index] = c[0, 3]
            y1[index] = c[1, 3]
            z1[index] = c[2, 3]
            index += 1

    plt.figure(3)
    plt.plot(x1, y1, 'b.')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('XY plocha')

    sel = 3024
    a1 = np.zeros((3, sel))
    b1 = np.zeros((3, sel))

    a1[0, :] = xy[:, 0]
    a1[1, :] = xy[:, 1]
    a1[2, :] = xy[:, 2]

    b1[0, :] = t[:, 0]
    b1[1, :] = t[:, 1]
    b1[2, :] = t[:, 2]

    model = Sequential()
    model.add(Dense(numnodes, input_dim=3, activation='relu'))
    model.add(Dense(3))

    model.compile(optimizer='adam', loss='mean_squared_error')
    model.fit(a1.T, b1.T, epochs=1000, validation_split=0.1, verbose=0)

    x1 = 17.33
    y1 = np.arange(-10, 11, 1)
    xy1 = np.zeros((21, 3))
    e = np.zeros(21)

    for i in range(21):
        aa = np.array([[x1, y1[i], 0]])
        th = model.predict(aa)

        a = DH(th[0, 0], np.pi / 2, 0, 0)
        b = np.dot(a, DH(th[0, 1], 0, 10, 0))
        c = np.dot(b, DH(th[0, 2], 0, 10, 0))

        xy1[i] = [c[0, 3], c[1, 3], c[2, 3]]
        e[i] = np.sqrt((c[0, 3] - x1) ** 2 + (c[1, 3] - y1[i]) ** 2 + (c[2, 3] - 0) ** 2)

    plt.figure(3)
    plt.plot(xy1[:, 0], xy1[:, 1], '*')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Vysledek')

    ee = np.sum(e)
    eee[i_n] = ee


plt.show()
print("Průměrné chyby pro jednotlivé konfigurace:")
for i, error in enumerate(eee):
    print(f"Konfigurace {i+1} s {numA[i]} uzly: {error/21}")