"""
Simulates points from a moving robot. 
Simulated points are expose dusing the PointBuffer API.
"""
import numpy as np
from open3d import *


class PointBuffer():
    """
    A point buffer accumulates points from a stream. 
    Points in the buffer are either valid or expired.
    """
    def __init__(self,stream):
        self.stream = stream
        self.stream.skip(300)
        self.points = []
        self.colors = []
        self.classes = []
        self.pcd = PointCloud()


    def get_frame(self):
        """Return the points that were just collected from the sensor"""
        data = next(self.stream)
        self.stream.skip(20)
        
        if data==None:
            self.stream.reset()
            return self.get_frame()
        points, colors, classes, _ = data
        self.points.append(points)
        self.colors.append(colors)
        self.classes.append(classes)
        return points, colors, classes


    def get_buffer_points(self):
        """Return points that are held in the point buffer.
        Some of these points may have expired."""
        pcd = PointCloud()
        pcd.points = Vector3dVector(np.vstack(self.points))
        pcd.colors = Vector3dVector(np.vstack(self.colors))
        return pcd


    def get_valid_points(self):
        """Return a point cloud containing valid points"""
        invalid_classes = self.stream.get_invalid_classes()
        valid_points = []
        valid_colors = []
        for i,cls in enumerate(self.classes):
            valid_indicies = np.logical_not(np.isin(cls, invalid_classes))
            valid_points.append(self.points[i][valid_indicies]) 
            valid_colors.append(self.colors[i][valid_indicies]) 
        pcd = PointCloud()
        pcd.points = Vector3dVector(np.vstack(valid_points))
        pcd.colors = Vector3dVector(np.vstack(valid_colors))
        return pcd


    def get_expired_points(self):
        """Return a pointcloud containing invalid points"""
        invalid_classes = self.stream.get_invalid_classes()
        invalid_points = []
        invalid_colors = []
        for i,cls in enumerate(self.classes):
            invalid_indicies = np.isin(cls, invalid_classes)
            invalid_points.append(self.points[i][invalid_indicies]) 
            invalid_colors.append(self.colors[i][invalid_indicies]) 
        pcd = PointCloud()
        pcd.points = Vector3dVector(np.vstack(invalid_points))
        pcd.colors = Vector3dVector(np.vstack(invalid_colors))
        return pcd
