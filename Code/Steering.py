import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time

r = 0.4

def rot(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0.0], [np.sin(theta), np.cos(theta), 0.0], [0.0, 0.0, 0.0]])


class CircPath():
    P_centre = np.zeros((3, 1))
    P_origin = np.zeros((3, 1))
    circle_vec = np.zeros((3, 1))
    __r = 0

    def __init__(self, P_origin, theta, r, orient=1):
        self.__r = r
        self.P_origin = P_origin
        self.P_centre = orient * r * np.array([[0.0, -1.0, 0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]]) @ np.array([np.cos(theta), np.sin(theta), 0]) + self.P_origin
        self.circle_vec = orient * np.array([0.0, 0.0, -r])


def tangent(CPs, CPe):
    d = CPe.P_centre - CPs.P_centre
    u_d = d / np.linalg.norm(d)
    if np.sign(CPs.circle_vec[2]) != np.sign(CPs.circle_vec[2]):
        alpha = np.arcsin(2*r/np.linalg.norm(d)) * np.sign(-CPs.circle_vec[2])
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
    length = (np.abs(ang_s) + np.abs(ang_e)) * r + np.linalg.norm(d)
    return (CPs.P_centre + r_st, CPe.P_centre + r_et, ang_s, ang_e, ang_s_1, ang_s_2, ang_e_1, ang_e_2, length)



def main():


    P_s = np.array([0, 0, 0])
    theta_s = np.deg2rad(90)

    P_e = np.array([-3, 0.5, 0])
    theta_e = np.deg2rad(90)

    plt.figure()
    plt.scatter(P_s[0], P_s[1], color='green')
    plt.scatter(P_e[0], P_e[1], color='red')

    for r in [0.4, 0.3]:

        CP_sl = CircPath(P_s, theta_s, r, +1)
        CP_sr = CircPath(P_s, theta_s, r, -1)

        CP_el = CircPath(P_e, theta_e, r, +1)
        CP_er = CircPath(P_e, theta_e, r, -1)

        CP_s_min, CP_s_max = CP_sl, CP_el
        length_min = np.inf
        for CP_s in [CP_sl, CP_sr]:
            for CP_e in [CP_el, CP_er]:
                P1, P2, a1, a2, ang_s_1, ang_s_2, ang_e_1, ang_e_2, length = tangent(CP_s, CP_e)
                if length < length_min:
                    length_min = length
                    CP_s_min = CP_s
                    CP_e_min = CP_e
                plt.scatter(P1[0], P1[1], color='orange', alpha=0.3)
                plt.scatter(P2[0], P2[1], color='orange', alpha=0.3)
                plt.plot([P1[0], P2[0]], [P1[1], P2[1]], linewidth=1.5, color='orange', alpha=0.3)
                wedge = patches.Arc(CP_s.P_centre[0:2], 2*r, 2*r, theta1=np.rad2deg(ang_s_1), theta2=np.rad2deg(ang_s_2), ec="orange", linewidth=1.5, fill=False, alpha=0.3)
                plt.gca().add_patch(wedge)
                wedge = patches.Arc(CP_e.P_centre[0:2], 2*r, 2*r, theta1=np.rad2deg(ang_e_1), theta2=np.rad2deg(ang_e_2), ec="orange", linewidth=1.5, fill=False, alpha=0.3)
                plt.gca().add_patch(wedge)
                print(f"angle_s = {a1}")
                print(f"angle_e = {a2}")
                print(f"length = {length}")

        P1, P2, a1, a2, ang_s_1, ang_s_2, ang_e_1, ang_e_2, length = tangent(CP_s_min, CP_e_min)
        plt.scatter(P1[0], P1[1], color='red')
        plt.scatter(P2[0], P2[1], color='red')
        plt.plot([P1[0], P2[0]], [P1[1], P2[1]], linewidth=3, color='red')
        wedge = patches.Arc(CP_s_min.P_centre[0:2], 2*r, 2*r, theta1=np.rad2deg(ang_s_1), theta2=np.rad2deg(ang_s_2), ec="red", linewidth=3, fill=False)
        plt.gca().add_patch(wedge)
        wedge = patches.Arc(CP_e_min.P_centre[0:2], 2*r, 2*r, theta1=np.rad2deg(ang_e_1), theta2=np.rad2deg(ang_e_2), ec="red", linewidth=3, fill=False)
        plt.gca().add_patch(wedge)

    plt.ylim(-5, 5)
    plt.xlim(-5, 5)
    plt.grid()
    plt.axis("equal")
    plt.show()


if __name__ == "__main__":
    main()