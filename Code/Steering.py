import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time

def rot(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0.0], [np.sin(theta), np.cos(theta), 0.0], [0.0, 0.0, 0.0]])

class CircPath():
    P_centre = np.zeros((3, 1))
    P_origin = np.zeros((3, 1))
    circle_vec = np.zeros((3, 1))
    r = 0

    def __init__(self, P_origin, theta, r, orient=1):
        self.r = r
        self.P_origin = P_origin
        self.P_centre = orient * r * np.array([[0.0, -1.0, 0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]]) @ np.array([np.cos(theta), np.sin(theta), 0]) + self.P_origin
        self.circle_vec = orient * np.array([0.0, 0.0, -r])


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
        ang_s = np.sign(-CPs.circle_vec[2]) * ((ang_s_2 - ang_s_1) % (2 * np.pi) + alpha)
        ang_e = np.sign(-CPe.circle_vec[2]) * ((ang_e_2 - ang_e_1) % (2 * np.pi) + alpha)
        length = np.abs(ang_s) * CPs.r + np.abs(ang_e) * CPe.r + np.linalg.norm(d)

        self.length = length
        self.ang_s = ang_s
        self.ang_e = ang_e
        self.P1 = CPs.P_centre + r_st
        self.P2 = CPe.P_centre + r_et
        self.ang_s_1 = ang_s_1
        self.ang_s_2 = ang_s_2
        self.ang_e_1 = ang_e_1
        self.ang_e_2 = ang_e_2

    def plot(self, ax, color='orange', linewidth=1.5, alpha=0.3):
        if self.length != np.inf:
            ax.scatter(self.P1[0], self.P1[1], color=color, alpha=alpha)
            ax.scatter(self.P2[0], self.P2[1], color=color, alpha=alpha)
            ax.plot([self.P1[0], self.P2[0]], [self.P1[1], self.P2[1]], linewidth=linewidth, color=color, alpha=alpha)
            wedge = patches.Arc(self.CP_s.P_centre[0:2], 2*self.CP_s.r, 2*self.CP_s.r, theta1=np.rad2deg(self.ang_s_1), theta2=np.rad2deg(self.ang_s_2), ec=color, linewidth=linewidth, fill=False, alpha=alpha)
            ax.add_patch(wedge)
            wedge = patches.Arc(self.CP_e.P_centre[0:2], 2*self.CP_e.r, 2*self.CP_e.r, theta1=np.rad2deg(self.ang_e_1), theta2=np.rad2deg(self.ang_e_2), ec=color, linewidth=linewidth, fill=False, alpha=alpha)
            ax.add_patch(wedge)


def main():
    P_s = np.array([0, 0, 0])
    theta_s = np.deg2rad(180)

    P_e = np.array([-1, 2, 0])
    theta_e = np.deg2rad(-90)

    plt.figure()
    plt.scatter(P_s[0], P_s[1], color='green')
    plt.scatter(P_e[0], P_e[1], color='red')

    CP_s_min, CP_s_max = None, None
    FP_min = None
    length_min = np.inf
    for r in [1.5, 1.4, 1.3, 1.2, 1.1, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2]:

        CP_sl = CircPath(P_s, theta_s, r, +1)
        CP_sr = CircPath(P_s, theta_s, r, -1)

        CP_el = CircPath(P_e, theta_e, r, +1)
        CP_er = CircPath(P_e, theta_e, r, -1)

        
        for CP_s in [CP_sl, CP_sr]:
            for CP_e in [CP_el, CP_er]:

                FP = FullPath(CP_s, CP_e)
                FP.plot(plt.gca())

                if FP.length < length_min:
                    length_min = FP.length
                    FP_min = FP


    FP_min.plot(plt.gca(), color="red", linewidth=3, alpha=1.0)



    plt.ylim(-5, 5)
    plt.xlim(-5, 5)
    plt.grid()
    plt.axis("equal")
    plt.show()


if __name__ == "__main__":
    main()