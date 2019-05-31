import os
import gym
import math
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet
import transforms3d
from . import SimRobot
from . import bullet_client
from .robot_models import Husky
from .robot_models import Turtlebot
from .config import URDF_ROOT
from .simulation_objects import create_object_by_name
from OpenGL.GL import glGetIntegerv, glGetDoublev
from OpenGL.GLU import gluUnProject
import random

RENDER_WIDTH = 960
RENDER_HEIGHT = 720

ZED_MIN_RANGE = 0.2
ZED_MAX_RANGE = 0.8
ZED_NOISE = 0.005

def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.
    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point
    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy


def make_camera(height, width, view_matrix, proj_matrix):
    # make camera matrix
    camera = {
        "class_name" : "PinholeCameraParameters",
        "extrinsic" : view_matrix,
        "intrinsic" : {
            "height": height,
            "width": width,
            "intrinsic_matrix": [
                    935.30743608719376,
                    0,
                    0,
                    0,
                    935.30743608719376,
                    0,
                    959.5,
                    539.5,
                    1
                ]
        },
        "version_major" : 1,
        "version_minor" : 0
    }
    return camera


class SimRobotEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }


    def __init__(self,
        urdfRoot=URDF_ROOT,
        world="y2e2/pybullet/world.sdf",
        floor="y2e2/pybullet/floor.urdf",
        actionRepeat=50,
        buildingScale=0.15,
        isEnableSelfCollision=True,
        isDiscrete=False,
        renders=False,
        reflection=False):

        self.world = world
        self.floor = floor
        self.timeStep = .01
        self.urdfRoot = urdfRoot
        self.actionRepeat = actionRepeat
        self.isEnableSelfCollision = isEnableSelfCollision
        self.observation = []
        self.ballUniqueId = -1
        self.robot = None               # The controlled robot
        self.buildingIds = []           # Each plane is given an id
        self.buildingScale = buildingScale
        self.width = 320
        self.height = 240
        self.reflection = reflection

        self.envStepCounter = 0
        self.renders = renders
        self.isDiscrete = isDiscrete
        if self.renders:
            self.physics = bullet_client.BulletClient(connection_mode=pybullet.GUI)
        else:
            self.physics = bullet_client.BulletClient()

        self.seed()
        observationDim = 2  # len(self.getExtendedObservation())
        observation_high = np.ones(observationDim) * 1000  # np.inf
        if isDiscrete:
            self.action_space = spaces.Discrete(9)
        else:
            action_dim = 2
            self._action_bound = 1
            action_high = np.array([self._action_bound] * action_dim)
            self.action_space = spaces.Box(-action_high, action_high)
        self.observation_space = spaces.Box(-observation_high, observation_high)
        self.viewer = None


    def reset(self):
        print("Resetting environment")
        floor_path = os.path.join(self.urdfRoot, self.floor)
        world_path = os.path.join(self.urdfRoot, self.world)
        base_orientation = pybullet.getQuaternionFromEuler([0, 0, 1.54])
        self.physics.resetSimulation()
        self.cam_yaw = 0

        print("Loading floor geometry from ",floor_path)
        print("Loading world geometry from ",world_path)
        self.floorIds = self.physics.loadURDF(floor_path, baseOrientation=base_orientation, globalScaling=self.buildingScale)
        self.buildingIds = self.physics.loadSDF(world_path, globalScaling=self.buildingScale)

        # Disable rendering while we load the robot. Enable reflection
        self.physics.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING,0)
        if self.reflection:
            self.physics.configureDebugVisualizer(pybullet.COV_ENABLE_PLANAR_REFLECTION,0)
        self.physics.configureDebugVisualizer(pybullet.COV_ENABLE_GUI,0)
        self.physics.configureDebugVisualizer(pybullet.COV_ENABLE_TINY_RENDERER,0)
        self.physics.setGravity(0, 0, -10)

        config = {}
        config['initial_pos'] = (0,0,0)
        config["target_pos"] = (1, 1, 1)
        config["resolution"] = None
        config["is_discrete"] = False

        #self.robot = SimRobot.SimRobot(self.physics, urdfRootPath=self.urdfRoot, timeStep=self.timeStep)
        self.robot = Turtlebot(config, self.physics)
        self.envStepCounter = 0
        for i in range(100):
            self.physics.stepSimulation()
        self.observation = self.getExtendedObservation()
        # Enable rendering
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)
        return np.array(self.observation)


    def transform_camera_to_absolute(self,pts):
        """
        Return the position of points in the global coordinate system
        @points is a column matrix of points in the camera coordinate system
        """
        #pos, orn = self.physics.getBasePositionAndOrientation(self.robot.racecarUniqueId)
        R1 = np.eye(3)#np.array([[0,0,1],[0,1,0],[1,0,0]])
        RY = np.eye(3)#np.array([[0,0,1],[0,1,0],[-1,0,0]])
        R2 = np.eye(3)#np.linalg.inv(transforms3d.quaternions.quat2mat(orn))
        #print(pos)
        #return np.dot(pts, R1)
        

        correct = np.dot(np.dot(pts, R1), R2.T)
        return np.dot(correct,RY)# - pos


    def get_absolute_position(self,x,y,z,t):
        """
        Get the absolute position of an object if it appears to
        be at position x,y,z, and rotation t from the car.
        """
        x = -x # Relative position is defined left=negative. Absolute is the opposite
        car_pos, car_orn = self.physics.getBasePositionAndOrientation(self.robot.racecarUniqueId)
        car_yaw = pybullet.getEulerFromQuaternion(car_orn)[2]
        car_point = (car_pos[0], car_pos[1])
        cone_point = (car_pos[0]+z, car_pos[1]+x)
        rotated_cone_point = rotate(car_point, cone_point, car_yaw)
        X = rotated_cone_point[1]
        Y = y
        Z = rotated_cone_point[0]
        return X,Y,Z,t


    def get_relative_position(self,X,Y,Z,t):
        """
        Inverse function of get_absolute_position
        """
        # Get car position
        car_pos, car_orn = self.physics.getBasePositionAndOrientation(self.robot.racecarUniqueId)
        car_yaw = pybullet.getEulerFromQuaternion(car_orn)[2]
        car_point = (car_pos[0], car_pos[1])
        # Get point relative to car
        rotated_object_point = (Z,X)
        object_point = rotate(car_point, rotated_object_point, -car_yaw)
        # Shift point relative to car
        object_point_shifted = (object_point[0]-car_pos[0], object_point[1]-car_pos[1])
        x = object_point_shifted[1]
        z = object_point_shifted[0]
        x = -x # Relative position is defined left=negative. Absolute is the opposite
        return x,Y,z,t


    def spawn_object_by_name(self, class_name, X, Y, Z):
        """
        Spawn an object at [x,y,z] relative to the car
        Dimensions are defined as follows:
            X: Horizontal offset
            Y: Vertical direction
            Z: Horizontal offset (distance from camera)
        """
        _, new_orn = self.physics.getBasePositionAndOrientation(self.robot.racecarUniqueId)
        new_pos = np.array([Z,X,Y])
        return create_object_by_name(self.physics, class_name, new_pos, new_orn)


    def move_car(self, x, y, yaw):
        """Move the car to a new position and orientation"""
        # Change the x,y position
        old_pos, old_orn = self.physics.getBasePositionAndOrientation(self.robot.racecarUniqueId)
        new_pos = (x, y, old_pos[2])
        # Change the yaw
        old_rot = pybullet.getEulerFromQuaternion(old_orn)
        new_rot = (old_rot[0], old_rot[1], yaw)
        new_orn = pybullet.getQuaternionFromEuler(new_rot)
        self.physics.resetBasePositionAndOrientation(self.robot.racecarUniqueId, new_pos, new_orn)
        self.cam_yaw = yaw


    def __del__(self):
        self.physics = 0


    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]


    #def getExtendedObservation(self):
        # TODO:  Add 12 angle ray-collision test (verify details)
        #self.observation = []  # self._racecar.getObservation()
        #carpos, carorn = self.physics.getBasePositionAndOrientation(self.robot.racecarUniqueId)
        #ballpos, ballorn = self.physics.getBasePositionAndOrientation(self.ballUniqueId)
        #invCarPos, invCarOrn = self.physics.invertTransform(carpos, carorn)
        #ballPosInCar, ballOrnInCar = self.physics.multiplyTransforms(invCarPos, invCarOrn, ballpos, ballorn)

        #self.observation.extend([ballPosInCar[0], ballPosInCar[1]])
        #return self.observation

    def getExtendedObservation(self):
        return
        carpos, carorn = self.physics.getBasePositionAndOrientation(self.robot.racecarUniqueId)
        carmat = self.physics.getMatrixFromQuaternion(carorn)
        ballpos, ballorn = self.physics.getBasePositionAndOrientation(self.ballUniqueId)
        invCarPos, invCarOrn = self.physics.invertTransform(carpos, carorn)
        ballPosInCar, ballOrnInCar = self.physics.multiplyTransforms(invCarPos, invCarOrn, ballpos, ballorn)
        dist0 = 0.3
        dist1 = 1.
        eyePos = [carpos[0] + dist0 * carmat[0], carpos[1] + dist0 * carmat[3], carpos[2] + dist0 * carmat[6] + 0.3]
        targetPos = [carpos[0] + dist1 * carmat[0], carpos[1] + dist1 * carmat[3],
                     carpos[2] + dist1 * carmat[6] + 0.3]
        up = [carmat[2], carmat[5], carmat[8]]
        viewMat = self.physics.computeViewMatrix(eyePos, targetPos, up)
        # viewMat = self._p.computeViewMatrixFromYawPitchRoll(carpos,1,0,0,0,2)
        # print("projectionMatrix:")
        # print(self._p.getDebugVisualizerCamera()[3])
        projMatrix = [0.7499999403953552, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0,
                      0.0, 0.0, -0.02000020071864128, 0.0]
        img_arr = self.physics.getCameraImage(width=self.width, height=self.height, viewMatrix=viewMat,
                                         projectionMatrix=projMatrix)
        rgb = img_arr[2]
        np_img_arr = np.reshape(rgb, (self.height, self.width, 4))
        self.observation = np_img_arr
        return self.observation

    def step(self, action):
        if self.renders:
            basePos, orn = self.physics.getBasePositionAndOrientation(self.robot.racecarUniqueId)
            # self.physics.resetDebugVisualizerCamera(1, 30, -40, basePos)

        if self.isDiscrete:
            fwd = [-1, -1, -1, 0, 0, 0, 1, 1, 1]
            steerings = [-0.6, 0, 0.6, -0.6, 0, 0.6, -0.6, 0, 0.6]
            forward = fwd[action]
            steer = steerings[action]
            realaction = [forward, steer]
        else:
            realaction = action

        self.robot.applyAction(realaction)
        for i in range(self.actionRepeat):
            self.physics.stepSimulation()
            if self.renders:
                time.sleep(self.timeStep)
            self.observation = self.getExtendedObservation()

            if self.termination():
                break
            self.envStepCounter += 1
        reward = self.reward()
        done = self.termination()
        # print("len=%r" % len(self._observation))

        return np.array(self.observation), reward, done, {}


    def render(self, width, height):
        # Move the camera with the base_pos
        base_pos, orn = self.physics.getBasePositionAndOrientation(self.robot.racecarUniqueId)
        cam_yaw = 180 * self.cam_yaw / math.pi + 90
        cam_yaw = cam_yaw % 360
        cam_dist = 1
        cam_roll = 0
        cam_pitch = 0

        view_matrix = self.physics.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=base_pos,
            distance=cam_dist,
            yaw=cam_yaw,
            pitch=cam_pitch,
            roll=cam_roll,
            upAxisIndex=2)
        proj_matrix = self.physics.computeProjectionMatrixFOV(
            fov=60, aspect=float(width) / height,
            nearVal=0.1, farVal=100.0)
        (_, _, px, _, _) = self.physics.getCameraImage(
            width=width, height=height, viewMatrix=view_matrix,
            projectionMatrix=proj_matrix, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)
        rgb_array = np.array(px)
        rgb_array = rgb_array[:, :, :3]
        return rgb_array


    def render_pointcloud(self, width, height, cam_yaw=0):
        """Return an image as 3D points"""
        # Move the camera with the base_pos
        base_pos, orn = self.physics.getBasePositionAndOrientation(self.robot.racecarUniqueId)

        cam_yaw = 180 * cam_yaw / math.pi + 90
        cam_yaw = cam_yaw % 360
        cam_dist = 1
        cam_roll = 0
        cam_pitch = 0

        base_pos = (base_pos[0], base_pos[1], base_pos[2]+1)
        view_matrix = self.physics.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=base_pos,
            distance=cam_dist,
            yaw=cam_yaw,
            pitch=cam_pitch,
            roll=cam_roll,
            upAxisIndex=2)

        view_matrix_reverse = self.physics.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=base_pos,
            distance=cam_dist,
            yaw=(cam_yaw+180)%360,
            pitch=cam_pitch,
            roll=cam_roll,
            upAxisIndex=2)

        proj_matrix = self.physics.computeProjectionMatrixFOV(
            fov=60, aspect=float(width) / height,
            nearVal=1, farVal=40.0)

        (_, _, rgb, depth, segmentation) = self.physics.getCameraImage(
            width=width, height=height, viewMatrix=view_matrix,
            projectionMatrix=proj_matrix, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)

        # Make image
        rgb_image = np.array(rgb)
        rgb_image = rgb_image[:, :, :3]

        # Make points
        viewport = (0, 0, width, height)
        matrixModelView  = view_matrix
        matrixProjection = proj_matrix

        # Make points slightly noise
        depth += np.random.normal(scale=ZED_NOISE, size=depth.shape)

        # Make camera params
        camera = make_camera(height, width, view_matrix_reverse, proj_matrix)

        # Output vectors
        pts = np.zeros((width*height, 3))
        pts_rgb = np.zeros((width*height, 3))
        pts_seg = np.zeros((width*height, ))

        for y in range(height):
            if y%10==0:
                pass
                #print('row:',y)
            for x in range(width):
                z = depth[y,x]
                if z==1.0: continue
                if z<ZED_MIN_RANGE or z>ZED_MAX_RANGE: continue
                i = x + y*width # Simple counter
                pts[i,:] = gluUnProject(x, y, z, matrixModelView, matrixProjection, viewport)
                pts_rgb[i,:] = rgb_image[y,x,:] / 255
                pts_seg[i] = segmentation[y,x]
        return pts, pts_rgb, pts_seg, camera


    def termination(self):
        return self.envStepCounter > 1000


    def reward(self):
        # Adapt the reward to:
        # 1 if target reached, else 0
        # -1 if wall collision
        closestPoints = self.physics.getClosestPoints(self.robot.racecarUniqueId, self.ballUniqueId, 10000)

        numPt = len(closestPoints)
        reward = -1000
        # print(numPt)
        if (numPt > 0):
            # print("reward:")
            reward = -closestPoints[0][8]
            # print(reward)
        return reward
