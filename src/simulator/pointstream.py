"""
Generate a stream of points
New points can be collected usibng stream.__next__()
"""

import os
import cv2
import time
import glob
import math
import random
import pybullet
import scipy.misc
import pandas as pd
from open3d import *
import collections
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from .sim.simulation.environment.simulation_env import SimRobotEnv


OBJECT_DETECTOR = "object_rcnn_trained"
FLOOR = "y2e2/pybullet/floor.urdf"
WORLD = "y2e2/pybullet/world.sdf"
RENDER_WIDTH = 640
RENDER_HEIGHT = 480
OUTPUTS = ["val.npz"]

OBJECTS = [
    "cone",
    "hard hat",
    "gloves",
    "bucket"
]



class SceneGenerator():

    def __init__(self, metadata_file, add_objects=True, render=True):
        """Create two different environments (before,after)"""
        self.metadata = pd.read_csv(metadata_file, header=0)
        self.it = self.metadata.iterrows()
        self.env = SimRobotEnv(renders=render, isDiscrete=False, world=WORLD, floor=FLOOR)
        self.env.reset()
        self.reset(add_objects=add_objects, probability=0.4)
        self.added_object_ids = []
        self.removed_object_ids = []


    def __iter__(self):
        """Returns itself as an iterator object"""
        return self


    def __next__(self):
        """Iterate along the trajectory, returning images and point clouds"""
        i,row = next(self.it)
        camera_x = row['camera_x']
        camera_y = row['camera_y']
        camera_yaw = row['camera_roll'] + math.pi # DATASET IS WRONG HERE
        self.env.move_car(x=camera_x, y=camera_y, yaw=camera_yaw)

        # Spawn an image if we are in spawn mode
        if self.add_objects and len(self.added_object_ids) and random.random()>self.probability:
            self.remove_object(random.choice(self.added_object_ids))
        if self.add_objects and random.random()>self.probability:
            self.spawn_object()
        points, rgb, classes, camera = self.env.render_pointcloud(RENDER_WIDTH, RENDER_HEIGHT, cam_yaw=camera_yaw)
        return points, rgb, classes, camera


    def skip(self,n):
        """
        Skip the iterator forward n steps
        """
        for i in range(n):
            next(self.it)


    def get_invalid_classes(self):
        """Return a list of object Ids that have been removed"""
        return self.removed_object_ids


    def spawn_object(self):
        """Spawn a random object in front of the car"""
        class_name = random.choice(OBJECTS)
        x = random.uniform(-1,1)
        y = 0
        z = random.uniform(-1,-3)
        X,Y,Z,T = self.env.get_absolute_position(x,y,z,t=0)
        obj = self.env.spawn_object_by_name(class_name, X, Y, Z)
        self.added_object_ids.append(obj.shapeId)


    def remove_object(self, object_id):
        self.added_object_ids.remove(object_id)
        self.removed_object_ids.append(object_id)
        self.env.physics.removeBody(object_id)


    def reset(self, add_objects=True, probability=0.3):
        self.add_objects = add_objects
        self.probability = probability
        self.it = self.metadata.iterrows()
        self.env.reset()








if __name__=="__main__":
    stream = Stream()
    for points in stream:
        print(points)

