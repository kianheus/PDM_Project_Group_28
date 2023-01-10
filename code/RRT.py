# -----------------------------------------------------------------------------
# Import needed packages
# -----------------------------------------------------------------------------

import numpy as np
import Steering as steer
from tqdm import tqdm, trange
from matplotlib import pyplot as plt
from matplotlib import patches
import time
import Approximator

# -----------------------------------------------------------------------------
# Define class that executes the RRT algorithm
# -----------------------------------------------------------------------------
'''
Generate a pose at position (x, y) and at an angle t in degrees
'''
def pose_deg(x, y, t):
    return np.array([x, y, np.deg2rad(t)])


'''
A single node in the RR tree.
A node has a given position/angle.
Each node also has a backreference to the edge leading to it, as well as the previous node visited.
Finally, each node has a set distance from the origin point.
'''
class Node():
    def __init__(self, pose : np.ndarray, distance_from_parent : float = 0.0, parent_node = None, parent_edge=None):
        self.pose = pose
        self.parent_node = parent_node
        self.children_nodes : list[Node] = []
        self.parent_edge = parent_edge
        self.distance_from_parent = distance_from_parent
        self.distance_from_origin = distance_from_parent
        if parent_node is not None:
            # print(f"{parent_node=}")
            parent_node.children_nodes.append(self)
            self.distance_from_origin += parent_node.distance_from_origin


'''
An edge is a connection between two nodes of the RR tree.
Each edge has a reference to a start node and an end node. Note that the end node will also have a reference to the start node and this edge.
Finally, the edge has a path object, which in this case is a Dubbins Path
'''
class Edge():
    def __init__(self, start_node : Node, path : steer.Path):
        self.start_node = start_node
        self.path = path
        self.end_node = Node(path.end_pose, self.path.length, self.start_node, self)



'''
The environment map that is being planned for is a collection of obstacles and also contains parameters that do not change over the lifetime of the map.
The map can be used to check for collisions, as well as generating random positions or poses.
'''
class Map():
    def __init__(self, obstacles : np.ndarray, vehicle_radius : float = None, workspace_center = None, workspace_size = None, consts = None):
        if consts is not None:
            if vehicle_radius is None:
                vehicle_radius = consts.vehicle_radius
            if workspace_center is None:
                workspace_center = consts.workspace_center
            if workspace_size is None:
                workspace_size = consts.workspace_size
        self.obstacles = obstacles
        self.vehicle_radius = vehicle_radius
        self.workspace_center = workspace_center
        self.workspace_size = workspace_size

        self.obstacle_1 = np.atleast_2d(self.obstacles[:,0] - self.obstacles[:,3]/2)
        self.obstacle_2 = np.atleast_2d(self.obstacles[:,0] + self.obstacles[:,3]/2)
        self.obstacle_3 = np.atleast_2d(self.obstacles[:,1] - self.obstacles[:,4]/2)
        self.obstacle_4 = np.atleast_2d( self.obstacles[:,1] + self.obstacles[:,4]/2)

    def collision_check_array(self, points : np.ndarray) -> np.ndarray:
        points = np.atleast_2d(points)
        points_x = np.atleast_2d(points[:,0]).T
        points_y = np.atleast_2d(points[:,1]).T
        a = points_x + self.vehicle_radius > self.obstacle_1
        a &= points_x - self.vehicle_radius < self.obstacle_2
        a &= points_y + self.vehicle_radius > self.obstacle_3
        a &= points_y - self.vehicle_radius < self.obstacle_4
        return a.any(axis=1)

    def collision_check(self, points : np.ndarray) -> bool:
        return self.collision_check_array(points).any()


    def collision_check_single(self, point : np.ndarray) -> bool:
        for obstacle in self.obstacles:
            if point[0] + self.vehicle_radius > obstacle[0] - obstacle[3]/2 and point[0] - self.vehicle_radius < obstacle[0] + obstacle[3]/2\
                and point[1] + self.vehicle_radius > obstacle[1] - obstacle[4]/2 and point[1] - self.vehicle_radius < obstacle[1] + obstacle[4]/2:
                return True
        return False
    
    

    def random_position(self) -> np.ndarray:
        collision = True
        while collision:
            new_xy = np.random.uniform(low=self.workspace_center-self.workspace_size/2, high=self.workspace_center+self.workspace_size/2, size = (1,2))[0]
            collision = self.collision_check(new_xy)
        return new_xy

    def random_pose(self) -> np.ndarray:
        # new_theta = (np.random.beta(2, 2) *2*np.pi - np.pi) % (2*np.pi)
        new_theta = np.random.uniform(low=-np.pi, high=np.pi)
        return np.hstack((self.random_position(), new_theta))

    def plot(self, ax : plt.Axes):
        for obstacle in self.obstacles:
            margin = 0.2
            ax.add_patch(patches.Rectangle((obstacle[0]-obstacle[3]/2-margin,obstacle[1]-obstacle[4]/2-margin), obstacle[3]+2*margin, obstacle[4]+2*margin, facecolor="lightgrey"))
        for obstacle in self.obstacles:
            ax.add_patch(patches.Rectangle((obstacle[0]-obstacle[3]/2,obstacle[1]-obstacle[4]/2), obstacle[3], obstacle[4], facecolor="darkred"))
            


'''
This class contains the RR tree.
The tree has a base node, which the tree is grown from.
The tree also has a list of edge objects, which can be used to backtrack through the tree.
The tree has a list of node poses (not node objects), which can be used to rapidly find the nearest node.
The tree also contains a collision map and parameters of the search
'''
class Tree():
    def __init__(self, map : Map, initial_pose : np.ndarray, turning_radius : float = None, collision_resolution : float = None, consts = None, local_planner = False):
        if consts is not None:
            if turning_radius is None:
                turning_radius = consts.turning_radius
            if collision_resolution is None:
                collision_resolution = consts.collision_resolution
        self.map = map
        self.base_node = Node(initial_pose)
        self.edges : list[Edge] = []
        self.nodes : list[Node] = [self.base_node]
        self.node_poses = np.array([initial_pose])
        self.node_distances = np.array([0])
        self.turning_radius = turning_radius
        self.collision_resolution = collision_resolution
        self.dummy_counter = 0
        self.DA = Approximator.DubbinsApproximator(turning_radius=turning_radius)
        
        if local_planner == True:
            
            #moving_obstacle = self.map.obstacles[-1,:]
            
            pose_angle = self.base_node.pose[-1]
            pose_x = self.map.workspace_center[0]
            pose_y = self.map.workspace_center[1] + 1.5
            new_pose1 = np.hstack((pose_x, pose_y, pose_angle))
            self.add_path_to(new_pose1, modify_angle=False)
            
            pose2_y = self.map.workspace_center[1] - 1.5
            new_pose2 = np.hstack((pose_x, pose2_y, pose_angle))
            self.add_path_to(new_pose2, modify_angle=False)
            


    def print(self):
        print(f"Tree:")
        print(f"{len(self.nodes)} nodes")
        print(f"{len(self.edges)} edges")

    '''
    Function to add a new node to the tree. Note that this function breaks the dubbins path up into segments, thus generating more nodes in the tree.
    '''
    def add_node(self, start_node : Node, path : steer.Path):
        node = start_node
        for segment in path.segments:
            path_new = steer.PathSimple(segment)
            new_edge = Edge(node, path_new)
            node = new_edge.end_node
            self.nodes.append(node)
            self.edges.append(new_edge)
            self.node_poses = np.append(self.node_poses, np.atleast_2d(new_edge.end_node.pose), axis = 0)
            self.node_distances = np.append(self.node_distances, new_edge.end_node.distance_from_origin)
        # print("Added new node")

    '''
    This function selects a random pose in the environment (which is not in colision) and connects it to the graph
    '''
    def grow_single(self, end_pose=None, informed = False, set_angle=None):
        if informed:
            valid_node = False
            while valid_node == False:
                best_distance = np.partition(self.node_distances, 1)[1]
                #best_distance = np.min(self.node_distances)
                print("Hello", best_distance)
                sample_new_pose = self.map.random_pose()
                distance_from_origin = steer.optimal_path(self.base_node.pose, sample_new_pose, self.turning_radius).length
                distance_to_end = steer.optimal_path(sample_new_pose, end_pose, self.turning_radius).length
                
                if distance_from_origin + distance_to_end < best_distance:
                    return self.add_path_to(sample_new_pose)
                    valid_node = True
                else:
                   sample_new_pose = self.map.random_pose() 
                   valid_node = False
    
        else:
            if set_angle is None:
                return self.add_path_to(self.map.random_pose())
            else:
                new_pose = self.map.random_pose()
                new_pose[2] = set_angle
                return self.add_path_to(new_pose, modify_angle=False)


    '''
    Grow the path until a valid path to the end_pose is found.
    Iteration is controlled using the iter parameter. By passing a trange(n), a loading bar is shown
    Iteration is cut off after some amount of seconds to prevent runnaway.
    The function returns true if a path has been found.
    '''
    def grow_to(self, end_pose : np.ndarray, iter = range(100), max_seconds = 180, star = True, finish = True, informed = False, set_angle=None, local_planner=False):
        close_time=time.time() + max_seconds
        added_node = True
        done = False
        neighbouring_node_ids = np.array([])
        for i in iter:
            if time.time()>close_time:
                print("Time limit met, stopping.")
                break
            if added_node:
                if star:
                    if neighbouring_node_ids.shape[0] > 0:  
                        self.rewire(neighbouring_node_ids)
                if finish:
                    done_single, path = self.connect_to_newest_node(end_pose)
                    done |= done_single
                if finish and done:
                    print("Found a path.")
                    break
            added_node, neighbouring_node_ids = self.grow_single(end_pose, informed=informed, set_angle=set_angle)
        if not finish:
            done, _ = self.add_path_to(end_pose, modify_angle=False, n_closest=100, i_break=40)
        return done

    def grow_blind(self, iter = range(100), max_seconds = 180):
        close_time=time.time() + max_seconds
        added_node = True
        neighbouring_node_ids = np.array([])
        for i in iter:
            if time.time()>close_time:
                print("Time limit met, stopping.")
                break
            if added_node:
                if neighbouring_node_ids.shape[0] > 0:  
                    self.rewire(neighbouring_node_ids)
                    
            added_node, neighbouring_node_ids = self.grow_single()
        return


    '''
    Finds a path from the origin to the given end pose.
    This function returns none if the given end pose is not a node in the tree.
    If a path exists, a steer.Path object is returned going from the origin to the end pose
    '''
    def path_to(self, end_pose):
        segments = []
        node = self.get_node(end_pose)

        if node is None:
            return None  # No path to end_pose found
        
        while node is not None:
            if node.parent_edge is not None:
                path : steer.Path = node.parent_edge.path
                segments.extend(path.segments)
            node = node.parent_node

        segments.reverse()
        path = steer.Path(segments)
        return path
    
    '''
    Tries to match the given pose to a node in the tree.
    Due to floating point errors, a tolerance value must be used.
    Returns None if no matching node exists.
    '''
    def get_node(self, pose):
        offset = self.node_poses - pose
        offset[:,2] += np.pi
        offset[:,2] %= 2 * np.pi
        offset[:,2] -= np.pi
        offset[:,2] /= 10
        distances = np.linalg.norm(offset, axis=1)
        idx_closest = np.argmin(distances)
        if distances[idx_closest] > 2e-3:
            # print(f"No matching node found. Closest node at {distances[idx_closest]}.")
            # print(f"{distances=}")
            # print(f"{offset=}")
            return None
        return self.nodes[idx_closest]


    '''
    Tries to connect a new_pose to the newest node in the tree
    '''
    def connect_to_newest_node(self, new_pose : np.ndarray):
        node = self.get_node(new_pose)
        path = steer.optimal_path(self.node_poses[-1], new_pose, self.turning_radius)
        new_path_is_shorter = False

        # if node is not None:
        #     new_path_is_shorter = (node.distance_from_origin > self.nodes[-1].distance_from_origin + path.length)
        #     if new_path_is_shorter:
                # print("new path is shorter")
        #         discrete_path = path.interpolate(d=self.collision_resolution)
        #         collision = self.map.collision_check(discrete_path)
        #         if not collision:
        #             # modify path leading up to newest node


        if node is None:
            discrete_path = path.interpolate(d=self.collision_resolution)
            collision = self.map.collision_check(discrete_path)
            if collision:
                return False, None
            # add path to tree
            self.add_node(self.nodes[-1], path)
            return True, path

        return True, node.parent_edge.path

    '''
    This function finds a path from a node on the tree to the new node.
    If this path is in collision, it is not added to the tree.
    Using an upper bound on the shortest path to a node (dubbins path), most nodes can be ignored when generating dubbins paths.
    '''
    def add_path_to(self, new_pose : np.ndarray, modify_angle=True, n_closest=25, i_break=5) -> bool:
        valid_indices = np.argsort(np.linalg.norm(self.node_poses[:,:2] - new_pose[:2], axis=1))[:n_closest] # Select 10 closest nodes


        path_dist_approx = []
        angle_random = new_pose[2]

        for idx in valid_indices:
            if modify_angle:
                displacement = (new_pose - self.node_poses[idx])[:2]
                angle_displacement = np.arctan2(displacement[1], displacement[0])
                angle = (angle_displacement + angle_random) % (np.pi * 2)
                new_pose[2] = angle
            # potential_steering_paths.append(steer.optimal_path(self.node_poses[idx], new_pose, self.turning_radius))
            path_dist_approx.append(self.DA.lookup(self.node_poses[idx], new_pose, Approximator.InterpolationType.Interpolated))

        shortest_path_ids = np.argsort([dist + self.node_distances[valid_indices][i] for i, dist in enumerate(path_dist_approx)])
        # shortest_path_ids = np.argsort([path.length + self.node_distances[valid_indices][i] for i, path in enumerate(potential_steering_paths)])
        
        for i, shortest_path_idx in enumerate(shortest_path_ids):
            if i>i_break:
                break
            steering_path = steer.optimal_path(self.node_poses[valid_indices][shortest_path_idx], new_pose, self.turning_radius)
            # steering_path = potential_steering_paths[shortest_path_idx]
            parent_coord_idx = valid_indices[shortest_path_idx]

            discrete_path = steering_path.interpolate(d=self.collision_resolution)

            collision = self.map.collision_check(discrete_path)
            if collision:
                continue

            if parent_coord_idx == 0:
                self.add_node(self.base_node, steering_path)
            else:
                self.add_node(self.edges[parent_coord_idx-1].end_node, steering_path)

            return True, valid_indices
        return False, valid_indices 
    

    """
    This function takes in the current pose and a list of indeces corresponding to all the nodes of the tree within a
    radius r of the current node. Then, it calculates the distance of the Dubin's path from the current node to all the nearby
    nodes and it connects the current node to the nearby node that is nearest (in Dubin distance) 
    """    
    def rewire(self, neighbouring_node_ids):
        
        added_node = self.nodes[-1]
        added_pose = self.node_poses[-1]
        
        for idx in neighbouring_node_ids:
            
            neighbouring_pose = self.node_poses[idx] # read the pose
            neighbouring_distance_origin = self.node_distances[idx] # read the previously recorded distance of the neighbour to the origin
            
            path = steer.optimal_path(neighbouring_pose, added_pose, self.turning_radius)
            
            new_neighbouring_distance_origin = added_node.distance_from_origin + path.length
            
            
            if new_neighbouring_distance_origin < neighbouring_distance_origin:
                #self.dummy_counter += 1
                #print(f"{self.dummy_counter=}")
                
                discrete_path = path.interpolate(d=self.collision_resolution)
                
                collision = self.map.collision_check(discrete_path)
                if collision:
                    continue

                # print(f"Rewiring node #{len(self.nodes)-1}")
                
                # Update the parameters of the neighbour we are rewiring to
                self.nodes[idx].parent_node.children_nodes.remove(self.nodes[idx]) # remove old parent
                self.nodes[idx].parent_node = added_node
                self.nodes[idx].parent_node.children_nodes.append(self.nodes[idx]) # assign new parent
                self.nodes[idx].parent_edge.start_node = added_node
                self.nodes[idx].parent_edge.path = path
                self.nodes[idx].distance_from_origin = new_neighbouring_distance_origin
                self.nodes[idx].distance_from_parent = path.length
                self.node_distances[idx] = self.nodes[idx].distance_from_origin

                for child in self.nodes[idx].children_nodes:
                    self.distance_update(self.nodes[idx], child)

                self.node_distances = np.array([node.distance_from_origin for node in self.nodes])



    def distance_update(self, parent : Node, child : Node):
        child.distance_from_origin = child.distance_from_parent + parent.distance_from_origin
        child.children_nodes
        for grandchild in child.children_nodes:
            self.distance_update(child, grandchild) 

def plot_pose(ax, pose, length=0.1, **kwargs):
    kwargs["linewidth"] = 3
    ax.scatter(pose[0], pose[1], **kwargs)
    kwargs.pop("s", None)
    ax.plot([pose[0], pose[0] + np.cos(pose[2])*length], [pose[1], pose[1] + np.sin(pose[2])*length], **kwargs)

def plot_points(ax, points, **kwargs):
    plt.scatter(points[:,0], points[:,1], c=range(points.shape[0]))