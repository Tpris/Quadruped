#
# Fortune Maeva et Tissot Priscilla
#

import numpy as np
import matplotlib.pyplot as plt


class LinearSpline:
    def __init__(self):
        self.T = []
        self.X = []

    def add_entry(self, t, x):
        i=0
        while i<len(self.T) and t > self.T[i]: i += 1
        if i==len(self.T):
            self.T.append(t)
            self.X.append(x)
        else:
            self.T.insert(i, t)
            self.X.insert(i, x)

    def interpolate(self, t):
        i=0
        while i<len(self.T) and t > self.T[i]: i += 1
        if i==0: return self.X[0]
        if i==len(self.T): return self.X[i-1]
        return self.X[i-1] + (t-self.T[i-1]) / (self.T[i] - self.T[i-1]) * (self.X[i] - self.X[i-1])

class LinearSpline3D:
    def __init__(self):
        self.X = LinearSpline()
        self.Y = LinearSpline()
        self.Z = LinearSpline()

    def add_entry(self, t, x, y ,z):
        self.X.add_entry(t,x)
        self.Y.add_entry(t,y)
        self.Z.add_entry(t,z)

    def interpolate(self, t):
        return (self.X.interpolate(t), \
                self.Y.interpolate(t), \
                self.Z.interpolate(t))


if __name__ == "__main__":
    spline = LinearSpline()
    spline.add_entry(0., 0.)
    spline.add_entry(0.5, 0.2)
    spline.add_entry(1.5, -0.4)
    spline.add_entry(2.3, 0.6)

    xs = np.arange(-0.1, 2.5, 0.1)
    ys = []
    for x in xs:
        ys.append(spline.interpolate(x))

    plt.plot(xs, ys)
    plt.show()
