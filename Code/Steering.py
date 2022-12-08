import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import itertools

def rot(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0.0], [np.sin(theta), np.cos(theta), 0.0], [0.0, 0.0, 0.0]])

class CircPath():
    P_centre = np.zeros((3, 1))
    P_origin = np.zeros((3, 1))
    circle_vec = np.zeros((3, 1))
    r = 0

    def __init__(self, P_origin, theta, r, orient=1):
        if P_origin.shape[0] == 2:
            P_origin = np.hstack((P_origin, 0.0))
        self.r = r
        self.P_origin = P_origin
        orient = np.sign(orient)
        self.P_centre = orient * r * np.array([[0.0, -1.0, 0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]]) @ np.array([np.cos(theta), np.sin(theta), 0]) + self.P_origin
        self.circle_vec = orient * np.array([0.0, 0.0, -r])

    def print(self, ax, alpha=1.0):
        circle = patches.Circle(self.P_centre, self.r, fill=False, alpha=alpha)
        ax.add_patch(circle)

class Circ3Path():
    def __init__(self, CPs, CPe, r, orient=1.0):
        self.CP_s = CPs
        self.CP_e = CPe
        self.r = r
        self.length = np.inf

        self.d = CPe.P_centre - CPs.P_centre
        self.m = (CPe.P_centre + CPs.P_centre) / 2
        u_d = self.d / np.linalg.norm(self.d)

        r4 = 2*r + CPe.r + CPs.r
        self.r4 = r4
        
        # Start and end circles are too far appart
        if np.linalg.norm(self.d) > r4:
            return
        
        # Start and end circles should have the same polarity
        if np.sign(CPs.circle_vec[2]) != np.sign(CPe.circle_vec[2]):
            return

        self.length = 0

        self.circle_vec = -np.sign(CPs.circle_vec) * r

        self.o = np.sqrt((r4) ** 2 - np.linalg.norm(self.d)**2) / 2 * (rot(np.deg2rad(90)*np.sign(self.circle_vec[2])) @ u_d)

        self.P_centre = self.m + orient * self.o
        self.P1 = (self.P_centre + CPs.P_centre) / 2
        self.P2 = (self.P_centre + CPe.P_centre) / 2

        self.r_P1 = self.P1 - self.P_centre
        self.r_P2 = self.P2 - self.P_centre

        self.ang_1 = np.arctan2(self.r_P1[1], self.r_P1[0])
        self.ang_2 = np.arctan2(self.r_P2[1], self.r_P2[0])

        if self.circle_vec[2] > 0:
            self.ang_1, self.ang_2 = self.ang_2, self.ang_1

        self.r_sP1 = self.P1 - self.CP_s.P_centre
        self.r_eP2 = self.P2 - self.CP_e.P_centre

        self.ang_s_2 = np.arctan2(self.r_sP1[1], self.r_sP1[0])
        self.ang_e_1 = np.arctan2(self.r_eP2[1], self.r_eP2[0])

        r_s = CPs.P_origin - CPs.P_centre
        r_e = CPe.P_origin - CPe.P_centre
        self.ang_s_1 = np.arctan2(r_s[1], r_s[0])
        if self.CP_s.circle_vec[2] > 0:
            self.ang_s_1, self.ang_s_2 = self.ang_s_2, self.ang_s_1
        self.ang_e_2 = np.arctan2(r_e[1], r_e[0])
        if self.CP_e.circle_vec[2] > 0:
            self.ang_e_1, self.ang_e_2 = self.ang_e_2, self.ang_e_1

        self.ang_s = np.sign(-self.CP_s.circle_vec[2]) * ((self.ang_s_2 - self.ang_s_1) % (2 * np.pi))
        self.ang_m = np.sign(-self.circle_vec[2]) * ((self.ang_2 - self.ang_1) % (2 * np.pi))
        self.ang_e = np.sign(-self.CP_e.circle_vec[2]) * ((self.ang_e_2 - self.ang_e_1) % (2 * np.pi))

        self.dist = [np.abs(self.ang_s) * self.CP_s.r, np.abs(self.ang_m) * self.r, np.abs(self.ang_e) * self.CP_e.r]
        self.curv = [-1 / self.CP_s.circle_vec[-1], -1 / self.circle_vec[-1], -1 / self.CP_e.circle_vec[-1]]

        self.length = sum(self.dist)

    def plot(self, ax, color='purple', linewidth=2, alpha=1.0, points=True):
        if self.length < np.inf:
            # circle = patches.Circle(self.m + self.o, self.r, fill=False)
            # ax.add_patch(circle)
            if points:
                ax.scatter(self.P1[0], self.P1[1], color=color, alpha=alpha)
                ax.scatter(self.P2[0], self.P2[1], color=color, alpha=alpha)
            wedge = patches.Arc(self.P_centre[0:2], 2*self.r, 2*self.r, theta1=np.rad2deg(self.ang_1), theta2=np.rad2deg(self.ang_2), ec=color, linewidth=linewidth, fill=False, alpha=alpha)
            ax.add_patch(wedge)
            wedge = patches.Arc(self.CP_s.P_centre[0:2], 2*self.CP_s.r, 2*self.CP_s.r, theta1=np.rad2deg(self.ang_s_1), theta2=np.rad2deg(self.ang_s_2), ec=color, linewidth=linewidth, fill=False, alpha=alpha)
            ax.add_patch(wedge)
            wedge = patches.Arc(self.CP_e.P_centre[0:2], 2*self.CP_e.r, 2*self.CP_e.r, theta1=np.rad2deg(self.ang_e_1), theta2=np.rad2deg(self.ang_e_2), ec=color, linewidth=linewidth, fill=False, alpha=alpha)
            ax.add_patch(wedge)
            
            # print(f"length (ccc) = {self.length}")

    def print(self):
        print(f"distance = {self.dist[0]}, curve = {self.curv[0]}")
        print(f"distance = {self.dist[1]}, curve = {self.curv[1]}")
        print(f"distance = {self.dist[2]}, curve = {self.curv[2]}")
        print(f"Total = {sum(self.dist)}")

class FullPath():
    def __init__(self, CPs, CPe):
        self.CP_s = CPs
        self.CP_e = CPe

        d = CPe.P_centre - CPs.P_centre
        u_d = d / np.linalg.norm(d)
        if np.sign(CPs.circle_vec[2]) != np.sign(CPe.circle_vec[2]):
            if (CPs.r + CPe.r) < np.linalg.norm(d):
                alpha = np.arcsin((CPs.r + CPe.r)/np.linalg.norm(d)) * np.sign(-CPs.circle_vec[2])
            else:
                self.length = np.inf
                return
        else:
            alpha = 0
        r_st = rot(alpha) @ np.cross(CPs.circle_vec, u_d)
        r_et = rot(alpha) @ np.cross(CPe.circle_vec, u_d)
        r_s = CPs.P_origin - CPs.P_centre
        r_e = CPe.P_origin - CPe.P_centre
        ang_s_1 = np.arctan2(r_s[1], r_s[0])
        ang_s_2 = np.arctan2(r_st[1], r_st[0])
        if CPs.circle_vec[2] > 0:
            ang_s_1, ang_s_2 = ang_s_2, ang_s_1
        ang_e_1 = np.arctan2(r_e[1], r_e[0])
        ang_e_2 = np.arctan2(r_et[1], r_et[0])
        if CPe.circle_vec[2] < 0:
            ang_e_1, ang_e_2 = ang_e_2, ang_e_1
        ang_s = np.sign(-CPs.circle_vec[2]) * ((ang_s_2 - ang_s_1) % (2 * np.pi))
        ang_e = np.sign(-CPe.circle_vec[2]) * ((ang_e_2 - ang_e_1) % (2 * np.pi))

        self.P1 = CPs.P_centre + r_st
        self.P2 = CPe.P_centre + r_et

        self.dist = [np.abs(ang_s) * CPs.r, np.linalg.norm(self.P1 - self.P2), np.abs(ang_e) * CPe.r]
        self.curv = [-1 / CPs.circle_vec[-1], 0, -1 / CPe.circle_vec[-1]]

        self.length = sum(self.dist)

        self.ang_s = ang_s
        self.ang_e = ang_e

        self.ang_s_1 = ang_s_1
        self.ang_s_2 = ang_s_2
        self.ang_e_1 = ang_e_1
        self.ang_e_2 = ang_e_2

        

    def plot(self, ax, color='orange', linewidth=1.5, alpha=1.0, points=True):
        if self.length != np.inf:
            if points:
                ax.scatter(self.P1[0], self.P1[1], color=color, alpha=alpha)
                ax.scatter(self.P2[0], self.P2[1], color=color, alpha=alpha)
            ax.plot([self.P1[0], self.P2[0]], [self.P1[1], self.P2[1]], linewidth=linewidth, color=color, alpha=alpha)
            wedge = patches.Arc(self.CP_s.P_centre[0:2], 2*self.CP_s.r, 2*self.CP_s.r, theta1=np.rad2deg(self.ang_s_1), theta2=np.rad2deg(self.ang_s_2), ec=color, linewidth=linewidth, fill=False, alpha=alpha)
            ax.add_patch(wedge)
            wedge = patches.Arc(self.CP_e.P_centre[0:2], 2*self.CP_e.r, 2*self.CP_e.r, theta1=np.rad2deg(self.ang_e_1), theta2=np.rad2deg(self.ang_e_2), ec=color, linewidth=linewidth, fill=False, alpha=alpha)
            ax.add_patch(wedge)
            # print(f"length (csc) = {self.length}")

    def print(self):
        print(f"distance = {self.dist[0]}, curve = {self.curv[0]}")
        print(f"distance = {self.dist[1]}, curve = {self.curv[1]}")
        print(f"distance = {self.dist[2]}, curve = {self.curv[2]}")
        print(f"Total = {sum(self.dist)}")


def optimal_path(P_s, theta_s, P_e, theta_e, radii):
    if P_s.shape[0] == 2:
        P_s = np.hstack((P_s, 0.0))
    if P_e.shape[0] == 2:
        P_e = np.hstack((P_e, 0.0))
    FP_opt = None
    length_min = np.inf
    for r in radii:

        CP_ss = [CircPath(P_s, theta_s, r, +1), CircPath(P_s, theta_s, r, -1)]
        CP_es = [CircPath(P_e, theta_e, r, +1), CircPath(P_e, theta_e, r, -1)]
        
        for CP_s, CP_e in itertools.product(CP_ss, CP_es):
            FP = FullPath(CP_s, CP_e)
            if FP.length < length_min:
                FP_opt = FP
                length_min = FP_opt.length
            C3P1 = Circ3Path(CP_s, CP_e, r, orient=-1)
            if C3P1.length < length_min:
                FP_opt = C3P1
                length_min = FP_opt.length
            C3P2 = Circ3Path(CP_s, CP_e, r, orient=+1)
            if C3P2.length < length_min:
                FP_opt = C3P2
                length_min = FP_opt.length

    return FP_opt

def plot_point(ax, P, theta, color, length=0.1):
    ax.scatter(P[0], P[1], color=color)
    ax.plot(P[0] + [0, np.cos(theta)*length], P[1] + [0, np.sin(theta)*length], color=color, linewidth=3)

def main():
    P_s = np.array([0, 0])
    theta_s = np.deg2rad(90)

    P_e = np.array([0.5, 0.5])
    theta_e = np.deg2rad(-90)

    plt.figure()
    ax = plt.gca()

    plot_point(ax, P_s, theta_s, 'green')
    plot_point(ax, P_e, theta_e, 'blue')

    # FP_opt = None
    # length_min = np.inf
    # for r in [1.5, 1.4, 1.3, 1.2, 1.1, 1.0, 0.9, 0.8, 0.7, 0.6]:

    #     CP_ss = [CircPath(P_s, theta_s, r, +1), CircPath(P_s, theta_s, r, -1)]
    #     CP_es = [CircPath(P_e, theta_e, r, +1), CircPath(P_e, theta_e, r, -1)]

    #     # CP_ss = [CircPath(P_s, theta_s, r, +1)]
    #     # CP_es = [CircPath(P_e, theta_e, r, +1)]

    #     # CP_ss = [CircPath(P_s, theta_s, r, -1)]
    #     # CP_es = [CircPath(P_e, theta_e, r, -1)]

    #     # for CP in CP_ss + CP_es:
    #     #     CP.print(ax, alpha=0.0)
        
    #     for CP_s, CP_e in itertools.product(CP_ss, CP_es):
    #         FP = FullPath(CP_s, CP_e)
    #         C3P1 = Circ3Path(CP_s, CP_e, r, orient=-1)
    #         C3P2 = Circ3Path(CP_s, CP_e, r, orient=1)

    #         FP.plot(ax, alpha=0.2)
    #         C3P1.plot(ax, alpha=0.1)
    #         C3P2.plot(ax, alpha=0.1)

    #         if FP.length < length_min:
    #             FP_opt = FP
    #             length_min = FP_opt.length
    #         if C3P1.length < length_min:
    #             FP_opt = C3P1
    #             length_min = FP_opt.length
    #         if C3P2.length < length_min:
    #             FP_opt = C3P2
    #             length_min = FP_opt.length

    # FP_opt.plot(ax, color="red", linewidth=3, alpha=0.8)
    # FP_opt.print()

    # print(f"Shortest path = {FP_opt.length}")
    op = optimal_path(P_s, theta_s, P_e, theta_e, [1.5, 1.4, 1.3, 1.2, 1.1, 1.0, 0.9, 0.8, 0.7, 0.6])
    op.print()

    op.plot(ax, color="red", linewidth=3, alpha=0.8, points=False)



    plt.ylim(-5, 5)
    plt.xlim(-5, 5)
    plt.grid()
    plt.axis("equal")
    plt.show()


if __name__ == "__main__":
    main()