import os
import math
import pybullet
import numpy as np
from .config import URDF_ROOT


class PhysicsObject():
	filepath = ""
	specularColor = [0.4, 0.4, 0.4]
	rgbaColor = [1, 1, 1, 1]
	scale = [0.1, 0.1, 0.1]
	shift = [0, -0.02, 1]
	orientation = [0,0,0]

	def __init__(self, physics, position, orientation, visual_only=True):
		"""Create a new object at position and orientation"""
		path = os.path.join(URDF_ROOT, self.filepath)
		orn = pybullet.getQuaternionFromEuler(self.orientation)
		collisionScale = self.scale

		# TODO: The visual shape and collision shape can be re-used by all createMultiBody instances (instancing)
		self.visualShapeId = physics.createVisualShape(shapeType=pybullet.GEOM_MESH, fileName=path, rgbaColor=self.rgbaColor, specularColor=self.specularColor, visualFramePosition=self.shift, visualFrameOrientation=orn, meshScale=self.scale)
		self.collisionShapeId = physics.createCollisionShape(shapeType=pybullet.GEOM_MESH, fileName=path, collisionFramePosition=self.shift, collisionFrameOrientation=orn, meshScale=self.scale)
		self.shapeId = physics.createMultiBody(baseMass=100, baseInertialFramePosition=[0,0,0], baseCollisionShapeIndex=self.collisionShapeId, baseVisualShapeIndex=self.visualShapeId, basePosition=position, useMaximalCoordinates=True)

		# For tracking the position of this object over time
		self._position_history = []
		self._orientation_history = []


	def move(self, X, Y, Z, orientation=None):
		"""
		Move an object to position X,Y,Z in the global coordinate system
		Vertical position and orientation are not changed

		Dimensions are defined as follows:
            X: Horizontal offset (left is negative)
            Y: Vertical direction
            Z: Horizontal offset (distance from camera)
		"""
		old_pos, new_orn = pybullet.getBasePositionAndOrientation(self.shapeId)
		new_pos = np.array([Z,X,old_pos[2]])
		if orientation is not None:
			new_orn = pybullet.getQuaternionFromEuler(orientation)
		pybullet.resetBasePositionAndOrientation(self.shapeId, new_pos, new_orn)


	def move_kalman(self, position, orientation=None):
		"""
		Move this object by an incremental amount
		"""
		if orientation is None:
			_, orientation = pybullet.getBasePositionAndOrientation(self.shapeId)
		self._position_history.append(position)
		self._orientation_history.append(orientation)
		# The new position is the mean of all positions
		new_pos = np.mean(self._position_history, axis=0)
		new_orn = np.mean(self._orientation_history, axis=0)
		# Move to average position
		pybullet.resetBasePositionAndOrientation(self.shapeId, new_pos, new_orn)



class PhysicsObjectSDF(PhysicsObject):
	filepath = ""

	def __init__(self, physics, position, orientation):
		"""Create a new object at position and orientation"""
		path = os.path.join(URDF_ROOT, self.filepath)
		self.shapeId, = physics.loadSDF(path)
		self._position_history = []
		self._orientation_history = []
		physics.resetBasePositionAndOrientation(self.shapeId, position, orientation)


class SafetyCone(PhysicsObjectSDF):
	filepath = "tools/construction_cone_small/model.sdf"


class Baseball(PhysicsObject):
	shift = [0, 0, 0.1]
	scale = [0.02, 0.02, 0.02]
	filepath = "objects/baseball/baseball.obj"


class CardboardBox(PhysicsObject):
	shift = [0, 0, 0.3]
	scale = [0.3, 0.3, 0.3]
	orientation = [math.pi/2, 0, 0]
	specularColor = [0.5, 0.5, 0.5, 1]
	rgbaColor = [1.0, 0.3, 0.3, 1]
	filepath = "objects/box/box.obj"


class Chair(PhysicsObject):
	shift = [0, 0, 0]
	scale = [0.03, 0.03, 0.03]
	orientation = [math.pi/2, 0, 0]
	specularColor = [0.5, 0.5, 0.5, 1]
	rgbaColor = [0.3, 0.3, 1.0, 1]
	filepath = "objects/chair/chair.obj"


class RoadBike(PhysicsObject):
	scale = [0.006, 0.006, 0.006]
	rgbaColor = [0.2, 0.2, 0.2, 1]
	specularColor = [0.9, 0.9, 0.9, 1]
	orientation = [math.pi/2, 0, 0]
	shift = [0, 0.3, 0]
	filepath = "objects/cycle/Cycle.obj"


class GasCan(PhysicsObject):
	scale = [0.1, 0.1, 0.1]
	rgbaColor = [1, 0.2, 0.2, 1]
	specularColor = [0.5, 0.5, 0.5, 1]
	shift = [0, 0, 0.05]
	orientation = [math.pi/4, 0, 0]
	filepath = "objects/gas can/gascanhp.obj"


class Gloves(PhysicsObject):
	scale = [0.005, 0.005, 0.005]
	rgbaColor = [0.3, 0.3, 0.9, 1]
	specularColor = [0.4, 0.4, 0.4, 1]
	shift = [0, 0, 0.05]
	orientation = [math.pi/2, 0, math.pi/2]
	filepath = "objects/gloves/glove.obj"


class Bucket(PhysicsObject):
	scale = [0.1, 0.1, 0.1]
	rgbaColor = [0.99, 0.99, 0.99, 1]
	specularColor = [0.4, 0.4, 0.4, 1]
	shift = [0, 0, 0.05]
	orientation = [math.pi/2, 0, math.pi/2]
	filepath = "objects/bucket/bucket.obj"


class RecycleBin(PhysicsObject):
	scale = [0.02, 0.02, 0.02]
	rgbaColor = [0.1, 0.1, 0.9, 1]
	specularColor = [0.4, 0.4, 0.4, 1]
	shift = [0, 0, 0.05]
	orientation = [0, 0, math.pi/2]
	filepath = "objects/recycle bin/officebin.obj"


class HardHat(PhysicsObject):
	scale = [0.002, 0.002, 0.002]
	rgbaColor = [1, 1, 0.7, 1]
	specularColor = [0.9, 0.9, 0.9, 1]
	shift = [0, 0, 0.05]
	orientation = [math.pi/2, 0, math.pi/2]
	filepath = "objects/hard hat/helmet.obj"


def create_object_by_name(physics, name, position, orientation):
	"""Create an object, and return the PhysicsObject instance"""
	name_to_class = {
		"baseball": Baseball,
		"cardboard box": CardboardBox,
		"chair": Chair,
		"road bike": RoadBike,
		"gas can": GasCan,
		"cone": SafetyCone,
		"gloves": Gloves,
		"bucket": Bucket,
		"recycle bin": RecycleBin,
		"hard hat": HardHat,
	}
	object_class = name_to_class[name]
	return object_class(physics, position, orientation)