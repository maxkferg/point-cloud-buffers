import math
import random
import scipy.misc
import pandas as pd
from open3d import *


OBJECT_DETECTOR = "object_rcnn_trained"
FLOOR = "y2e2/pybullet/floor.urdf"
WORLD = "y2e2/pybullet/world.sdf"
RENDER_WIDTH = 640
RENDER_HEIGHT = 480

OUTPUTS = ["val.npz"]

OBJECTS = [
    "cone",
    "gas can",
    "hard hat",
    "gloves",
    "bucket"
]



class SceneGenerator():

    def __init__(self, env, metadata_file, render=True):
        """Create two different environments (before,after)"""
        self.metadata = pd.read_csv(metadata_file, header=0)
        self.it = self.metadata.iterrows()
        self.env = env(renders=render, isDiscrete=False, world=WORLD, floor=FLOOR)
        self.env.reset()
        self.reset(add_objects=False, probability=0)
        self.shapes = []


    def __iter__(self):
        """Returns itself as an iterator object"""
        return self


    def __next__(self):
        """Iterate along the trajectory, returning images and point clouds"""
        for i in range(5):
            next(self.it)
        i,row = next(self.it)
        camera_x = row['camera_x']
        camera_y = row['camera_y']
        camera_yaw = row['camera_roll'] + math.pi # DATASET IS WRONG HERE
        self.env.move_car(x=camera_x, y=camera_y, yaw=camera_yaw)

        # Spawn an image if we are in spawn mode
        if self.add_objects and random.random()>self.probability:
            self.spawn_object()
        points, rgb, camera = self.env.render_pointcloud(RENDER_WIDTH, RENDER_HEIGHT, cam_yaw=camera_yaw)
        return points, rgb, camera


    def skip(self,n):
        """
        Skip the iterator forward n steps
        """
        for i in range(n):
            next(self.it)


    def get_full_pointcloud(self, n=None, hop=0):
        """
        Iterate through n points and return the pull pointcloud
        Points are positioned in the global coordinate system
        Stops early if we run out of data
        """
        i = 0
        points = []
        colors = []
        for p, c, _ in self:
            points.append(p)
            colors.append(c)
            i += 1
            if n is not None and i>n:
                break
            self.skip(hop)

        pcd = PointCloud()
        pcd.points = Vector3dVector(np.vstack(points))
        pcd.colors = Vector3dVector(np.vstack(colors))
        return pcd


    def spawn_object(self):
        """Spawn a random object in front of the car"""
        class_name = random.choice(OBJECTS)
        x = random.uniform(-1,1)
        y = 0
        z = random.uniform(-1,-3)
        X,Y,Z,T = self.env.get_absolute_position(x,y,z,t=0)
        obj = self.env.spawn_object_by_name(class_name, X, Y, Z)
        self.shapes.append(obj.shapeId)


    def reset(self, add_objects=False, probability=0.1):
        self.add_objects = add_objects
        self.probability = probability
        self.it = self.metadata.iterrows()
        self.env.reset()
