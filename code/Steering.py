import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import itertools
from collections.abc import Iterable
import pygame

# Helper funciton to generate rotation matrix (3x3) for rotating theta radians about the z axis
def rot(theta) -> np.ndarray:
    return np.array([[np.cos(theta), -np.sin(theta), 0.0], [np.sin(theta), np.cos(theta), 0.0], [0.0, 0.0, 0.0]])

### Classes defining individual segments, which can be lines or arcs

class Segment():
    length = np.inf
    curvature = 0
    point_start = np.array([])
    point_end = np.array([])

    def __init__(self):
        raise NotImplementedError()

    def interpolate_single(self, b):
        raise NotImplementedError()

    def plot(self, ax, **kwargs):
        raise NotImplementedError()
    
    def interpolate_multi(self, bb):
        out = []
        for b in bb:
            out.append(self.interpolate_single(b))
        return np.array(out)

    def interpolate(self, n=100):
        self.interpolate(np.linspace(0.0, 1.0, n))

class Line(Segment):
    def __init__(self, s, e):
        self.point_start = s
        self.point_end = e
        self.length = np.linalg.norm(self.point_end - self.point_start)
        self.curvature = 0.0

    def interpolate_single(self, b):
        return self.point_start + b * (self.point_end - self.point_start)

    def plot(self, ax, **kwargs):
        kwargs.pop("s", None)
        ax.plot([self.point_start[0], self.point_end[0]], [self.point_start[1], self.point_end[1]], **kwargs)

    def plot_pygame(self, workspace, workspace_size, color):
        # Transform points from Mujoco ref frame (origin at workspace center)
        # to pygame ref frame (origin at top left corner)
        point_start_pg = to_pygame_coords(self.point_start, workspace_size) 
        point_end_pg = to_pygame_coords(self.point_end, workspace_size) 
        
        pygame.draw.line(workspace, color, point_start_pg, point_end_pg)

class Arc(Segment):

    def __init__(self, centre, radius, radius_vector_start, radius_vector_end):
        self.signed_radius = radius
        self.radius = np.abs(radius)
        self.curvature = 1/radius
        self.point_centre = centre

        self.radius_vector_start = radius_vector_start
        self.radius_vector_end = radius_vector_end
        self.point_start = self.point_centre + self.radius_vector_start
        self.point_end = self.point_centre + self.radius_vector_end
        self.update_angles()


    def from_origin(origin, theta, radius):
        if origin.shape[0] == 2:
            origin = np.hstack((origin, 0.0))
        point_centre = radius * np.array([[0.0, -1.0, 0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]]) @ np.array([np.cos(theta), np.sin(theta), 0]) + origin
        radius_vector_origin = origin - point_centre
        arc = Arc(point_centre, radius, radius_vector_origin, radius_vector_origin)
        arc.radius_vector_origin = radius_vector_origin
        return arc
        

    def set_start(self, radius_vector_start):
        self.radius_vector_start = radius_vector_start
        self.update_angles()

    def set_end(self, radius_vector_end):
        self.radius_vector_end = radius_vector_end
        self.update_angles()

    def update_angles(self):
        self.angle_end = np.arctan2(self.radius_vector_end[1], self.radius_vector_end[0])
        self.point_end = self.point_centre + self.radius_vector_end
        self.angle_start = np.arctan2(self.radius_vector_start[1], self.radius_vector_start[0])
        self.point_start = self.point_centre + self.radius_vector_start

        ang_s, ang_e = self.angle_start, self.angle_end
        if self.signed_radius < 0:
            ang_s, ang_e = ang_e, ang_s

        self.angle = np.sign(-self.signed_radius) * (((ang_e  - ang_s)) % (2*np.pi))

        self.length = np.abs(self.angle) * self.radius


    def vector(self) -> np.ndarray:
        return np.array([0.0, 0.0, self.signed_radius])

    def interpolate_single(self, b):

        if np.sign(self.signed_radius) < 0:
            b = 1 - b

        b *= np.sign(-self.signed_radius)
        ang_s, ang_e = self.angle_start, self.angle_end
        if self.signed_radius < 0:
            ang_s, ang_e = ang_e, ang_s
        ang = b * self.angle + ang_s  # angle of point in arc
        delta = np.array([np.cos(ang), np.sin(ang)]) * self.radius
        point = self.point_centre[0:2] + delta
        return point

    def plot(self, ax, **kwargs):
        kwargs = {(key if key != 'color' else 'ec'): value for key, value in kwargs.items()}
        kwargs.pop("s", None)
        ang_s, ang_e = self.angle_start, self.angle_end
        if self.signed_radius < 0:
            ang_s, ang_e = ang_e, ang_s
        ax.add_patch(patches.Arc(self.point_centre[0:2],
                                    2*self.radius, 2*self.radius,
                                    theta1=np.rad2deg(ang_s),
                                    theta2=np.rad2deg(ang_e), fill=False, **kwargs))

    def plot_pygame(self, workspace, workspace_size, color):
        
        ang_s, ang_e = self.angle_start, self.angle_end  
        if self.signed_radius < 0:
            ang_s, ang_e = ang_e, ang_s
            
        
        center_pg = to_pygame_coords(self.center, workspace_size)
        
        rect_left = center_pg[0] - self.radius  
        rect_top = center_pg[1] - self.radius
        rect_width = self.radius*2
        rect_height = self.radius*2

        rect = (rect_left, rect_top, rect_width, rect_height)
        pygame.draw.arc(workspace, color, rect, ang_s, ang_e)


### Classes defining a path, which is just a collection of segments

class Path():
    def __init__(self, segments=None):
        self.length = 0
        if segments is not None:
            self.segments = segments
        for segment in self.segments:
            self.length += segment.length

    def print(self):
        if len(self.segments) == 0:
            print("Empty Path")
        else:
            print(f"start position = {self.segments[0].point_start}")
            for segment in self.segments:
                print(f"distance     = {segment.length:.04}, curve = {segment.curvature:.04}")
            print(f"end position = {self.segments[-1].point_end}")
            print(f"Total = {self.length}")

    def plot(self, ax, endpoint=False, **kwargs):
        for segment in self.segments:
            segment.plot(ax, **kwargs)
        if endpoint:
            point_end, angle_end = self.interpolate_angles(n=2)
            plot_point(ax, point_end[-1], angle_end[-1], **kwargs)

    def plot_interpolate(self, ax, n=100, d=None):
        points = self.interpolate(n, d)
        ax.scatter(points[:,0], points[:,1], c=range(points.shape[0]), cmap='viridis')

    def interpolate_single(self, a):
        assert(a <= 1.0)
        assert(a >= 0.0)
        d = a * self.length  # the distance along the total path
        d_cumulative = np.cumsum([segment.length for segment in self.segments]) # cumulative distance traveled
        d_cumulative_padded = np.hstack((0.0, d_cumulative))
        for i, (segment, d_c, d_cp) in enumerate(zip(self.segments, d_cumulative, d_cumulative_padded)):
            if d <= d_c:
                b = (d-d_cp) / segment.length
                return segment.interpolate_single(b)
        return None

    def interpolate_multi(self, aa):
        points = []
        for a in aa:
            points.append(self.interpolate_single(a)[0:2])
        return np.array(points)

    def interpolate(self, n=100, d=None):
        if d is not None:
            n = (int) (np.round(self.length / d) + 1)
        return self.interpolate_multi(np.linspace(0.0, 1.0, n))
    
    def interpolate_angles(self, n=100, d=None, fdt=0.0001):
        if d is not None:
            n = (int) (np.ceil(self.length / d) + 1)
        dt = fdt * self.length / (n - 1)
        aa = np.linspace(0.0, 1.0-dt, n)
        bb = aa + dt
        points_a = self.interpolate_multi(aa)
        points_b = self.interpolate_multi(bb)
        diff = points_b - points_a
        mid = (points_a + points_b) / 2
        angles = np.arctan2(diff[:,1], diff[:,0])
        return (mid, angles)

    def interpolate_poses(self, n=100, d=None, fdt=0.0001):
        mid, angles = self.interpolate_angles(n, d, fdt)
        return np.hstack((mid, np.expand_dims(angles, axis=1)))

class PathTST(Path):
    def __init__(self, point_start, angle_start, radius_start, point_end, angle_end, radius_end):
        self.start_pose = np.hstack((point_start, angle_start))
        self.end_pose = np.hstack((point_end, angle_end))

        arc_start : Arc = Arc.from_origin(point_start, angle_start, radius_start)
        arc_end : Arc =   Arc.from_origin(point_end, angle_end, radius_end)

        d = arc_end.point_centre - arc_start.point_centre
        norm_d : float = np.linalg.norm(d)
        u_d : np.ndarray = d / norm_d

        if np.sign(arc_start.signed_radius) != np.sign(arc_end.signed_radius):
            if (arc_start.radius + arc_end.radius) < np.linalg.norm(d):
                alpha = np.arcsin((arc_start.radius + arc_end.radius) / norm_d) * np.sign(arc_start.signed_radius)
            else:
                self.length = np.inf
                return
        else:
            alpha = 0

        r_st = rot(alpha) @ np.cross(u_d, arc_start.vector())
        r_et = rot(alpha) @ np.cross(u_d, arc_end.vector())

        arc_start.set_end(r_st)
        arc_end.set_start(r_et)

        self.segments = []
        self.segments.append(arc_start)
        self.segments.append(Line(arc_start.point_end, arc_end.point_start))
        self.segments.append(arc_end)

        super().__init__()

    def optimal(point_start, angle_start, radius_start, point_end, angle_end, radius_end) -> Path:
        opt = None
        opt_len = np.inf
        for sign_s in [-1, 1]:
            for sign_e in [-1, 1]:
                path = PathTST(point_start, angle_start, sign_s * radius_start, point_end, angle_end, sign_e * radius_end)
                if opt_len > path.length:
                    opt = path
                    opt_len = opt.length
        return opt

class PathTTT(Path):
    def __init__(self, point_start, angle_start, radius_start, point_end, angle_end, radius_end, radius_middle):
        self.start_pose = np.hstack((point_start, angle_start))
        self.end_pose = np.hstack((point_end, angle_end))

        arc_start = Arc.from_origin(point_start, angle_start, radius_start)
        arc_end =   Arc.from_origin(point_end, angle_end, radius_end)

        self.d = arc_end.point_centre - arc_start.point_centre
        self.m = (arc_end.point_centre + arc_start.point_centre) / 2
        norm_d = np.linalg.norm(self.d)
        u_d = self.d / norm_d

        self.r4 = 2*np.abs(radius_middle) + arc_start.radius + arc_end.radius

        self.length = np.inf
        
        # Start and end circles are too far appart
        if norm_d > self.r4:
            return
        
        # Start and end circles should have the same polarity
        if np.sign(arc_start.signed_radius) != np.sign(arc_end.signed_radius):
            return

        self.length = 0

        signed_radius = -np.abs(radius_middle) * np.sign(arc_start.signed_radius)

        self.o = np.sqrt(self.r4** 2 - norm_d**2) / 2 * (rot(np.deg2rad(90)*np.sign(signed_radius)) @ u_d)

        middle_point_centre = self.m + np.sign(radius_middle) * self.o
        self.P1 = (middle_point_centre + arc_start.point_centre) / 2
        self.P2 = (middle_point_centre + arc_end.point_centre) / 2

        self.r_P1 = self.P1 - middle_point_centre
        self.r_P2 = self.P2 - middle_point_centre

        arc_start.set_end(-self.r_P1)
        arc_end.set_start(-self.r_P2)

        arc_middle = Arc(middle_point_centre, signed_radius, self.r_P1, self.r_P2)

        self.segments = []
        self.segments.append(arc_start)
        self.segments.append(arc_middle)
        self.segments.append(arc_end)

        super().__init__()


    def optimal(point_start, angle_start, radius_start, point_end, angle_end, radius_end, radius_middle) -> Path:
        opt = None
        opt_len = np.inf
        assert(np.sign(radius_start) == np.sign(radius_end))
        for sign_s in [-1, 1]:
            for sign_m in [-1, 1]:
                path = PathTTT(point_start, angle_start, sign_s * radius_start, point_end, angle_end, sign_s * radius_end, sign_m * radius_middle)
                if opt_len > path.length:
                    opt = path
                    opt_len = opt.length
        return opt

class PathSimple(Path):
    def __init__(self, segment):
        self.segments = [segment]
        super().__init__()
        points, angles = self.interpolate_angles(n=2)

        self.start_pose = np.hstack((points[0], angles[0]))
        self.end_pose = np.hstack((points[-1], angles[-1]))
        

# Function that generates the shortest path from start to end with a specified turning radius
# Returns a Path object
def optimal_path(point_start, point_end, radius) -> Path:
    P_s = np.squeeze(point_start)[:2]
    theta_s = np.squeeze(point_start)[2]
    P_e = np.squeeze(point_end)[:2]
    theta_e = np.squeeze(point_end)[2]
    path1 = PathTTT.optimal(P_s, theta_s, radius, P_e, theta_e, radius, radius)
    path2 = PathTST.optimal(P_s, theta_s, radius, P_e, theta_e, radius)
    opt_len = np.inf
    opt = None
    for path in [path1, path2]:
        if path is not None:
            if path.length < opt_len:
                opt = path
                opt_len = opt.length
    return opt

# Helper function to plot a point with a direction
def plot_point(ax, P, theta, length=0.1, **kwargs):
    kwargs["linewidth"] = 3
    ax.scatter(P[0], P[1], **kwargs)
    kwargs.pop("s", None)
    ax.plot(P[0] + [0, np.cos(theta)*length], P[1] + [0, np.sin(theta)*length], **kwargs)

def to_pygame_coords(point, window_size):
    
    """
    This function coverts the given point coordinates which are given with 
    respect to the center of the window to the reference frame used in pygame.
    This pygame reference frame has its origin in the top left edge of the window
    """
    x_offset = window_size[0]/2
    y_offset = window_size[1]/2
    
    x = point[0]
    y = point[1]
    
    if y > 0:
        y_new = y_offset - y
    else:
        y_new = y_offset + abs(y)

    if x > 0:
        x_new = x_offset + x
    else:
        x_new = x_offset - abs(x)
    new_point = [x_new, y_new]
    return new_point

### Some main functions for testing

def main():
    P_s = np.array([0, 0])
    theta_s = np.deg2rad(90)

    P_e = np.array([0.5, 2.5])
    theta_e = np.deg2rad(-90)

    plt.figure()
    ax = plt.gca()

    plot_point(ax, P_s, theta_s, 'green')
    plot_point(ax, P_e, theta_e, 'blue')

    op = optimal_path(P_s, theta_s, P_e, theta_e, 0.6)
    op.print()

    op.plot_interpolate(ax, n=5)
    op.plot(ax, color="red", linewidth=3, alpha=0.8)

    plt.ylim(-5, 5)
    plt.xlim(-5, 5)
    plt.grid()
    plt.axis("equal")
    plt.show()

def main2():
    l = Line(np.array([0.0, 0.2]), np.array([1.0, -0.5]))
    p = l.interpolate_multi([0.0, 0.5, 1.0])
    print(f"{p=}")

def main3():
    P_s = np.array([0, 0])
    theta_s = np.deg2rad(90)

    P_e = np.array([0.2, 0.2])
    theta_e = np.deg2rad(90)

    plt.figure()
    ax = plt.gca()

    plot_point(ax, P_s, theta_s, 'green')
    plot_point(ax, P_e, theta_e, 'blue')

    path = optimal_path(P_s, theta_s, P_e, theta_e, 0.2)
    # path.plot_interpolate(ax, d=0.1)
    points, angles = path.interpolate_angles(30)
    print(f"{points=}")
    print(f"{angles=}")
    for p, a in zip(points, angles):
        plot_point(ax, p, a, length=0.02)
    path.print()
    path.plot(ax, color="red", linewidth=1, alpha=0.5)

    print(f"{points[0,:]} == {P_s}")
    print(f"{points[-1,:]} == {P_e}")
    print(f"{angles[0]} == {theta_s}")
    print(f"{angles[-1]} == {theta_e}")

    plt.ylim(-5, 5)
    plt.xlim(-5, 5)
    plt.grid()
    plt.axis("equal")
    plt.show()


if __name__ == "__main__":
    main3()