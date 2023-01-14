# Import needed packages
import numpy as np
from tqdm import trange
from scipy.interpolate import interpn
import enum

# Import from custom files
import Steering as steer
import RRT

shape = (31, 31, 31)

class InterpolationType(enum.Enum):
   UpperBound = 0
   Interpolated = 1
   Nearest = 2

class DubbinsApproximator():
    def __init__(self, turning_radius=1.0, generate=False):
        self.turning_radius = turning_radius
        self.xx= np.hstack((np.geomspace(-51, -1.000001, (shape[0] - 1)//2) + 1, 0, np.geomspace(1.000001, 51, (shape[0] - 1)//2)-1))
        self.yy= np.hstack((np.geomspace(-51, -1.000001, (shape[1] - 1)//2) + 1, 0, np.geomspace(1.000001, 51, (shape[1] - 1)//2)-1))
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
                    path = steer.optimal_path(RRT.pose_deg(0, 0, 0), RRT.pose_deg(self.xx[x], self.yy[y], self.tt[t]), 1.0)
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


def test():
    turning_radius_g = 1.0

    DA = DubbinsApproximator(turning_radius_g)

    workspace_center = np.array([0, 0]) # Coordinate center of workspace
    workspace_size = np.array([20, 20]) # Dimensions of workspace

    env_map = RRT.Map(np.array([[0, 0, 0, 0, 0, 0, 0]]), 0.1, workspace_center, workspace_size)

    err_approx = []

    for _ in trange(1000):
        start_pose = env_map.random_pose()
        end_pose = env_map.random_pose()
        
        length_approx = DA.lookup(start_pose, end_pose, InterpolationType.Interpolated)

        length = steer.optimal_path(start_pose, end_pose, turning_radius_g).length

        err_approx.append(length_approx - length)

    print(f"Error - min  = {min(err_approx)}")
    print(f"Error - mean = {sum(err_approx)/len(err_approx)}")
    print(f"Error - std  = {np.std(np.array(err_approx))}")
    print(f"Error - max  = {max(err_approx)}")


def generate():
    DubbinsApproximator(generate=True)


if __name__ == "__main__":
    # generate()
    test()