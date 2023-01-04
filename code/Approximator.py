import numpy as np
import Steering as steer

from tqdm import trange
from matplotlib import pyplot as plt
# from scipy.interpolate import interpn
import enum


shape = (21, 21, 21)
turning_radius_g = 0.8

class InterpolationType(enum.Enum):
   UpperBound = 0
   Interpolated = 1
   Nearest = 2

class DubbinsApproximator():
    def __init__(self, turning_radius=1.0, generate=False):
        self.turning_radius = turning_radius
        self.xx= np.hstack((np.geomspace(-51, -1.00001, (shape[0] - 1)//2) + 1, 0, np.geomspace(1.00001, 51, (shape[0] - 1)//2)-1))
        self.yy= np.hstack((np.geomspace(-51, -1.00001, (shape[1] - 1)//2) + 1, 0, np.geomspace(1.00001, 51, (shape[1] - 1)//2)-1))
        self.tt = np.linspace(0, 360, shape[2])
        if generate:
            self.generate()
        else:
            self.lookup_table = np.load("dubbins_approximation.npy")

    def generate(self):
        z = np.zeros(shape)
        for x in trange(len(self.xx)):
            for y in range(len(self.yy)):
                for t in range(len(self.tt)):
                    path = steer.optimal_path(RRT.pose_deg(0, 0, 0), RRT.pose_deg(self.xx[x], self.yy[y], self.tt[t]), turning_radius)
                    z[x,y,t] = path.length if path is not None else 0.0
                    z[x,y,t] -= np.linalg.norm(np.array([self.xx[x], self.yy[y]]))

        np.save("dubbins_approximation.npy", z)
        self.lookup_table = z

    def lookup(self, start_pose, end_pose, typ : InterpolationType):
        delta_pose = steer.transform(start_pose, end_pose)
        delta_pose[:2] /= self.turning_radius
        result = 0

        if typ == InterpolationType.Nearest:
            xi = np.argmin(np.abs(np.array(self.xx)-delta_pose[0]))
            yi = np.argmin(np.abs(np.array(self.yy)-delta_pose[1]))
            ti = np.argmin(np.abs(np.array(self.tt)-delta_pose[2]))
            result = self.lookup_table[xi, yi, ti] + np.linalg.norm(delta_pose[:2])

        if typ == InterpolationType.Interpolated:
            result = interpn((self.xx,self.yy,self.tt), self.lookup_table, delta_pose.T)[0] + np.linalg.norm(delta_pose[:2])

        if typ == InterpolationType.UpperBound:
            raise NotImplementedError()

        return result * self.turning_radius


def main2():
    import RRT

    DA = DubbinsApproximator(turning_radius_g)

    workspace_center = np.array([0, 0]) # Coordinate center of workspace
    workspace_size = np.array([30, 30]) # Dimensions of workspace

    env_map = RRT.Map([], 0.1, workspace_center, workspace_size)

    err_approx = []

    for _ in trange(1000):
        start_pose = env_map.random_pose()
        end_pose = env_map.random_pose()
        
        length_approx = DA.lookup(start_pose, end_pose, InterpolationType.Nearest)


        length = steer.optimal_path(start_pose, end_pose, turning_radius_g).length

        err_approx.append(length_approx - length)

    print(f"Error - min  = {min(err_approx)}")
    print(f"Error - mean = {sum(err_approx)/len(err_approx)}")
    print(f"Error - std  = {np.std(np.array(err_approx))}")
    print(f"Error - max  = {max(err_approx)}")




# def main():
#     turning_radius = 1.0

#     workspace_center = np.array([0, 0]) # Coordinate center of workspace
#     workspace_size = np.array([30, 30]) # Dimensions of workspace

#     env_map = RRT.Map([], 0.1, workspace_center, workspace_size)

#     # shape = (81, 81, 81)

#     # xx = np.linspace(-50, 50, shape[0])
#     # yy = np.linspace(-50, 50, shape[1])

#     xx= np.hstack((np.geomspace(-51, -0.9999, (shape[0] - 1)//2) + 1, 0, np.geomspace(1.0001, 51, (shape[0] - 1)//2)-1))
#     yy= np.hstack((np.geomspace(-51, -0.9999, (shape[1] - 1)//2) + 1, 0, np.geomspace(1.0001, 51, (shape[1] - 1)//2)-1))
#     tt = np.linspace(0, 360, shape[2])

    

#     generate = False

#     if generate:
#         z = np.zeros(shape)

#         for x in trange(len(xx)):
#             for y in range(len(yy)):
#                 for t in range(len(tt)):
#                     path = steer.optimal_path(RRT.pose_deg(0, 0, 0), RRT.pose_deg(xx[x], yy[y], tt[t]), turning_radius)
#                     z[x,y,t] = path.length if path is not None else 0.0
#                     z[x,y,t] -= np.linalg.norm(np.array([xx[x], yy[y]]))

#         np.save("dubbins_approximation.npy", z)
#     else:
#         z = np.load("dubbins_approximation.npy")

#     # zz = [steer.optimal_path(pose_deg(0, 0, 0), pose_deg(i, j, k), 1.0) for k in tqdm(t) for j in y for i in x]
#     # z = np.array([path.length if path is not None else 0.0 for path in zz])

#     X, Y = np.meshgrid(xx, yy)
#     # Z = z.reshape(21, 21, 21)

#     print(f"{X.shape=}")
#     print(f"{Y.shape=}")
#     print(f"{z.shape=}")
#     print(f"{np.max(z)=}")

#     fig, axs = plt.subplots(1, 1)

#     c = axs.pcolor(X, Y, z[:,:,5].T)
#     fig.colorbar(c, ax=axs)
#     # plt.axes("equal")

#     err = []
#     errx = []
#     erry = []

#     # for _ in trange(10000):
#     #     start_pose = env_map.random_pose()
#     #     end_pose = env_map.random_pose()

#     #     start_position = start_pose.copy()
#     #     start_position[2] = 1
#     #     end_position = end_pose.copy()
#     #     end_position[2] = 1

#     #     theta = start_pose[2]

#     #     transform = np.array([[np.cos(theta), -np.sin(theta), start_position[0]],[np.sin(theta), np.cos(theta), start_position[1]],[0, 0, 1]])

#     #     start_position_t = np.linalg.inv(transform) @ start_position
#     #     end_position_t = np.linalg.inv(transform) @ end_position

#     #     start_pose_t = start_position_t
#     #     start_pose_t[2] = (start_pose[2] - theta) % (2 * np.pi)
#     #     end_pose_t = end_position_t
#     #     end_pose_t[2] = (end_pose[2] - theta) % (2 * np.pi)

#     #     # print(f"{start_pose=}")
#     #     # print(f"{start_pose_t=}")
#     #     # print(f"{end_pose=}")
#     #     # print(f"{end_pose_t=}")
        
#     #     pose = env_map.random_pose()


#     #     length_1 = steer.optimal_path(RRT.pose_deg(0, 0, 0), end_pose_t, turning_radius).length
#     #     length_2 = steer.optimal_path(start_pose, end_pose, turning_radius).length

#     #     # print(f"{length_1=}")
#     #     # print(f"{length_2=}")

#     #     assert(np.abs(length_1 - length_2) < 1e-6)


#     # exit()

#     for _ in trange(1000):
#         start_pose = env_map.random_pose()
#         end_pose = env_map.random_pose()
        
#         pose = steer.transform(start_pose, end_pose)

#         length = steer.optimal_path(RRT.pose_deg(0, 0, 0), end_pose_t, turning_radius).length

#         xi = np.argmin(np.abs(np.array(xx)-pose[0]))
#         yi = np.argmin(np.abs(np.array(yy)-pose[1]))
#         ti = np.argmin(np.abs(np.array(tt)-pose[2]))
#         val_y = z[xi, yi, ti] + np.linalg.norm(np.array([xx[xi], yy[yi]]))

        
#         x_index_1 = np.argmax((np.array(xx)-pose[0]) > 0)
#         x_index_0 = x_index_1 - 1

#         y_index_1 = np.argmax((np.array(yy)-pose[1]) > 0)
#         y_index_0 = y_index_1 - 1

#         t_index_1 = np.argmax((np.deg2rad(np.array(tt))-pose[2]) > 0)
#         t_index_0 = t_index_1 - 1
        
#         # print(f"{diff_x > 0}")
#         # print(f"{diff_x < 0}")

#         val_max = -np.inf
#         for x_index in [x_index_0, x_index_1]:
#             for y_index in [y_index_0, y_index_1]:
#                 for t_index in [t_index_0, t_index_1]:
#                     val = z[x_index, y_index, t_index] + np.linalg.norm(np.array([xx[x_index], yy[y_index]]))
#                     if val > val_max:
#                         val_max = val
#                     # print(f"{val=}")

#         # print(f"upper bound =  {val_max}")
#         # print(f"upper bound =  {np.linalg.norm(pose[:2]) + 7*np.pi/3 * turning_radius}")
#         # print(f"actual value = {length}")

#         err.append(val_max - length)
#         erry.append(val_y - length)
#         errx.append(np.linalg.norm(pose[:2]) + 7*np.pi/3 * turning_radius - length)
        
#         # print(f"{pose=}: {length=}, {val_max=}")
        

#         # print(f"{xx[x_index]}, {yy[y_index]}, {tt[t_index]}")
#         # print(f"{xx[x_index]}, {yy[y_index]}, {tt[t_index]}")
#         # print(f"x is in: ({xx[x_index_0]}, {xx[x_index_1]})")
#         # print(f"y is in: ({yy[y_index_0]}, {yy[y_index_1]})")
#         # print(f"t is in: ({tt[t_index_0]}, {tt[t_index_1]})")

#     print(f"Error - min  = {min(err)}")
#     print(f"Error - mean = {sum(err)/len(err)}")
#     print(f"Error - std  = {np.std(np.array(err))}")
#     print(f"Error - max  = {max(err)}")


#     print(f"Conservative - min  = {min(errx)}")
#     print(f"Conservative - mean = {sum(errx)/len(errx)}")
#     print(f"Conservative - std  = {np.std(np.array(errx))}")
#     print(f"Conservative - max  = {max(errx)}")

#     print(f"Nearest - min  = {min(erry)}")
#     print(f"Nearest - mean = {sum(erry)/len(erry)}")
#     print(f"Nearest - std  = {np.std(np.array(erry))}")
#     print(f"Nearest - max  = {max(erry)}")

#     # print(f"{errx=}")

#     # plt.imshow(z[:,:,0],origin='lower',interpolation='bilinear')
#     plt.show()


if __name__ == "__main__":
    main2()
    # pass