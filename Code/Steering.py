import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

r = 0.4

def outer_tangent(P_sx, c_sx, P_ex, c_ex, P_s, P_e):
    d = P_ex - P_sx
    u_d = d / np.linalg.norm(d)
    r_st = np.cross(c_sx, u_d)
    r_et = np.cross(c_ex, u_d)
    r_s = P_s - P_sx
    r_e = P_e - P_ex
    ang_s = np.sign(-c_sx[2]) * ((np.sign(-c_sx[2]) * (np.arctan2(r_st[1], r_st[0]) - np.arctan2(r_s[1], r_s[0]))) % (2 * np.pi))
    ang_e = np.sign(-c_ex[2]) * ((np.sign(c_ex[2]) * (np.arctan2(r_et[1], r_et[0]) - np.arctan2(r_e[1], r_e[0]))) % (2 * np.pi))
    return (P_sx + r_st, P_ex + r_et, ang_s, ang_e)


def inner_tangent(P_sx, c_sx, P_ex, c_ex, P_s, P_e):
    d = P_ex - P_sx
    u_d = d / np.linalg.norm(d)
    alpha = np.arcsin(2*r/np.linalg.norm(d)) * np.sign(-c_sx[2])
    r_st = rot(alpha) @ np.cross(c_sx, u_d)
    r_et = rot(alpha) @ np.cross(c_ex, u_d)
    r_s = P_s - P_sx
    r_e = P_e - P_ex
    ang_s = np.sign(-c_sx[2]) * ((np.sign(-c_sx[2]) * (np.arctan2(r_st[1], r_st[0]) - np.arctan2(r_s[1], r_s[0]))) % (2 * np.pi) + alpha)
    ang_e = np.sign(-c_ex[2]) * ((np.sign(c_ex[2]) * (np.arctan2(r_et[1], r_et[0]) - np.arctan2(r_e[1], r_e[0]))) % (2 * np.pi) + alpha)
    return (P_sx + r_st, P_ex + r_et, ang_s, ang_e)


def rot(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 0]])

def main():


    P_s = np.array([0, 0, 0])
    theta_s = np.deg2rad(0)
    u_s = np.array([np.cos(theta_s), np.sin(theta_s), 0])
    P_sl = r * np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]]) @ u_s + P_s
    c_sl = np.array([0, 0, -r])
    P_sr = r * np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 0]]) @ u_s + P_s
    c_sr = np.array([0, 0, r])

    
    
    P_e = np.array([-1, 2, 0])
    theta_e = np.deg2rad(30)
    u_e = np.array([np.cos(theta_e), np.sin(theta_e), 0])
    P_el = r * np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]]) @ u_e + P_e
    c_el = np.array([0, 0, -r])
    P_er = r * np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 0]]) @ u_e + P_e
    c_er = np.array([0, 0, r])

    
    # d = P_er - P_sr
    # u_d = d / np.linalg.norm(d)
    # r_st = np.cross(c_sr, u_d)
    # r_et = np.cross(c_er, u_d)



    plt.figure()
    plt.scatter(P_s[0], P_s[1], color='green')
    plt.scatter(P_e[0], P_e[1], color='green')

    plt.scatter(P_sl[0], P_sl[1], color='blue')
    plt.scatter(P_sr[0], P_sr[1], color='blue')
    plt.scatter(P_el[0], P_el[1], color='blue')
    plt.scatter(P_er[0], P_er[1], color='blue')

    # P1, P2, a1, a2 = outer_tangent(P_sr, c_sr, P_er, c_er, P_s, P_e)
    # # P1 = P_sr + r_st
    # plt.scatter(P1[0], P1[1], color='red')
    # # P2 = P_er + r_et
    # plt.scatter(P2[0], P2[1], color='red')
    # plt.plot([P1[0], P2[0]], [P1[1], P2[1]], color='red')
    # print(f"angle_s = {a1}")
    # print(f"angle_e = {a2}")

    # P1, P2, a1, a2 = outer_tangent(P_sl, c_sl, P_el, c_el, P_s, P_e)
    # # P1 = P_sr + r_st
    # plt.scatter(P1[0], P1[1], color='red')
    # # P2 = P_er + r_et
    # plt.scatter(P2[0], P2[1], color='red')
    # plt.plot([P1[0], P2[0]], [P1[1], P2[1]], color='red')
    # print(f"angle_s = {a1}")
    # print(f"angle_e = {a2}")

    # P1, P2, a1, a2 = outer_tangent(P_sl, c_sl, P_el, c_el, P_s, P_e)
    # # P1 = P_sr + r_st
    # plt.scatter(P1[0], P1[1], color='red')
    # # P2 = P_er + r_et
    # plt.scatter(P2[0], P2[1], color='red')
    # plt.plot([P1[0], P2[0]], [P1[1], P2[1]], color='red')
    # print(f"angle_s = {a1}")
    # print(f"angle_e = {a2}")

    # P1, P2, a1, a2 = inner_tangent(P_sr, c_sr, P_el, c_el, P_s, P_e)
    # # P1 = P_sr + r_st
    # plt.scatter(P1[0], P1[1], color='red')
    # # P2 = P_er + r_et
    # plt.scatter(P2[0], P2[1], color='red')
    # plt.plot([P1[0], P2[0]], [P1[1], P2[1]], color='red')
    # print(f"angle_s = {a1}")
    # print(f"angle_e = {a2}")

    P1, P2, a1, a2 = inner_tangent(P_sl, c_sl, P_er, c_er, P_s, P_e)
    # P1 = P_sr + r_st
    plt.scatter(P1[0], P1[1], color='red')
    # P2 = P_er + r_et
    plt.scatter(P2[0], P2[1], color='red')
    plt.plot([P1[0], P2[0]], [P1[1], P2[1]], color='red')
    wedge = patches.Wedge(P1[0:2], r, 30, 270, ec="red", fill=False)
    plt.gca().add_patch(wedge)
    print(f"angle_s = {a1}")
    print(f"angle_e = {a2}")

    circle = patches.Circle(P_sl, r, fill=False)
    plt.gca().add_patch(circle)
    circle = patches.Circle(P_sr, r, fill=False)
    plt.gca().add_patch(circle)
    circle = patches.Circle(P_el, r, fill=False)
    plt.gca().add_patch(circle)
    circle = patches.Circle(P_er, r, fill=False)
    plt.gca().add_patch(circle)
    # patches.append(circle)

    plt.ylim(-5, 5)
    plt.xlim(-5, 5)
    plt.grid()
    plt.axis("equal")
    plt.show()


if __name__ == "__main__":
    main()