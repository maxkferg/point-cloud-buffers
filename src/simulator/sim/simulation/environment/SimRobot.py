# This is based on the pybullet racecar.py robot

import os
import numpy as np
import copy
import math

class SimRobot:
    def __init__(self, bullet_client, urdfRootPath='assets/', timeStep=0.01):
        self.urdfRootPath = urdfRootPath
        self.timeStep = timeStep
        self.physics = bullet_client
        self.reset()

    def reset(self):
        urdf = os.path.join(self.urdfRootPath, "turtlebot.urdf")
        print("Loading husky from ", urdf)
        car = self.physics.loadURDF(urdf, [1.2, +1.5, .2], useFixedBase=False)
        self.racecarUniqueId = car
        for wheel in range(self.physics.getNumJoints(car)):
            self.physics.setJointMotorControl2(car, wheel, self.physics.VELOCITY_CONTROL, targetVelocity=0, force=0)
            self.physics.getJointInfo(car, wheel)

            c = self.physics.createConstraint(car, 9, car, 11, jointType=self.physics.JOINT_GEAR, jointAxis=[0, 1, 0],
                                              parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
            self.physics.changeConstraint(c, gearRatio=1, maxForce=10000)

            c = self.physics.createConstraint(car, 10, car, 13, jointType=self.physics.JOINT_GEAR, jointAxis=[0, 1, 0],
                                              parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
            self.physics.changeConstraint(c, gearRatio=-1, maxForce=10000)

            c = self.physics.createConstraint(car, 9, car, 13, jointType=self.physics.JOINT_GEAR, jointAxis=[0, 1, 0],
                                              parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
            self.physics.changeConstraint(c, gearRatio=-1, maxForce=10000)

            c = self.physics.createConstraint(car, 16, car, 18, jointType=self.physics.JOINT_GEAR, jointAxis=[0, 1, 0],
                                              parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
            self.physics.changeConstraint(c, gearRatio=1, maxForce=10000)

            c = self.physics.createConstraint(car, 16, car, 19, jointType=self.physics.JOINT_GEAR, jointAxis=[0, 1, 0],
                                              parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
            self.physics.changeConstraint(c, gearRatio=-1, maxForce=10000)

            c = self.physics.createConstraint(car, 17, car, 19, jointType=self.physics.JOINT_GEAR, jointAxis=[0, 1, 0],
                                              parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
            self.physics.changeConstraint(c, gearRatio=-1, maxForce=10000)

            c = self.physics.createConstraint(car, 1, car, 18, jointType=self.physics.JOINT_GEAR, jointAxis=[0, 1, 0],
                                              parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
            self.physics.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)
            c = self.physics.createConstraint(car, 3, car, 19, jointType=self.physics.JOINT_GEAR, jointAxis=[0, 1, 0],
                                              parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
            self.physics.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)

            self.steeringLinks = [0, 2]
            self.maxForce = 200
            self.nMotors = 2
            self.motorizedwheels = [8, 15]
            self.speedMultiplier = 20.
            self.steeringMultiplier = 0.5

    def getActionDimension(self):
        return self.nMotors

    def getObservationDimension(self):
        return len(self.getObservation())

    def getObservation(self):
        observation = []
        pos, orn = self.physics.getBasePositionAndOrientation(self.racecarUniqueId)

        observation.extend(list(pos))
        observation.extend(list(orn))

        return observation

    def applyAction(self, motorCommands):
        targetVelocity = motorCommands[0] * self.speedMultiplier
        steeringAngle = motorCommands[1] * self.steeringMultiplier

        for motor in self.motorizedwheels:
            self.physics.setJointMotorControl2(self.racecarUniqueId, motor, self.physics.VELOCITY_CONTROL,
                                               targetVelocity=targetVelocity, force=self.maxForce)
        for steer in self.steeringLinks:
            self.physics.setJointMotorControl2(self.racecarUniqueId, steer, self.physics.POSITION_CONTROL,
                                               targetPosition=steeringAngle)
