"""
Define training datasets for point age task
"""
import os
import numpy as np
import open3d as o3d
from simulator.pointstream import SceneGenerator
from simulator.buffer import PointBuffer
import MinkowskiEngine as ME
import open3d as o3d
from torch.utils.data import Dataset
#from simulator.pointstream import SceneGenerator
#from simulator.buffer import PointBuffer


def crop_points(pcd, min_bound, max_bound):
    """Return two numpy arrays, points and colors"""
    points = np.array(pcd.points)
    colors = np.array(pcd.colors)
    keep = np.logical_and(
        np.any(points>=min_bound, axis=1),
        np.any(points<=max_bound, axis=1))
    return points[keep], colors[keep]


def quantize_points(points, feats, voxel_size):
    """
    Transform points to a sparse vector.
    Return coords, quantized_points, feats
    """
    quantized_coords = np.floor(points / voxel_size)
    inds = ME.utils.sparse_quantize(quantized_coords)
    quantized_coords = quantized_coords[inds]
    quantized_points = points[inds]
    quantized_feats = feats[inds]
    return quantized_coords, quantized_points, quantized_feats


def add_time_dimension(quantized_coords, t):
    """
    Add the time dimension to coords
    """
    n = quantized_coords.shape[0]
    coords = t*np.ones((n,4))
    coords[:,:3] = quantized_coords
    return coords


class SimulatorDataset(Dataset):
    """
    Generate a (coord, feat, label) triplet
    Output:
        coord: A Nx4 tensor with dimensions x,y,z,t
               The full pointcloud resides at dimension t=0
               New points are assigned to the t=1 dimension
        feat: A Nx3 tensor containing R,G,and B channels
        label: A Nx1 tensor containing 1 for valid points and 0 for expired points
    """ 

    def __init__(self, voxel_size):
        """
        """
        metadata_file = "simulator/dataset/metadata.csv"
        self.point_stream = SceneGenerator(metadata_file)
        self.point_buffer = PointBuffer(self.point_stream)
        self.voxel_size = voxel_size

    def __len__(self):
        return 500

    def __getitem__(self, idx):

        # Prefetch the first frame
        if not len(self.point_buffer.points):
            self.point_buffer.get_frame()

        # Get the frame of new points at time t
        points, colors, _ = self.point_buffer.get_frame()
        min_bound = np.min(points)
        max_bound = np.max(points)
        new_coords, _, new_feats = quantize_points(points, colors, self.voxel_size)
        new_labels = np.ones((new_coords.shape[0], 1))

        # Get the valid points at time t
        valid_pcd = self.point_buffer.get_buffer_points()
        valid_points, valid_feats = crop_points(valid_pcd, min_bound, max_bound)
        valid_coords, _, valid_feats = quantize_points(valid_points, valid_feats, self.voxel_size)
        valid_labels = np.ones((valid_coords.shape[0], 1))

        # Get the expired points at time t
        expired_pcd = self.point_buffer.get_expired_points()
        #if len(expired_pcd.points):
        expired_points, expired_feats = crop_points(expired_pcd, min_bound, max_bound)
        if len(expired_points):
            expired_coords, _, expired_feats = quantize_points(expired_points, expired_feats, self.voxel_size)
            expired_labels = np.zeros((expired_coords.shape[0], 1))
        else:
            expired_coords = np.zeros((0,3))
            expired_feats = np.zeros((0,3))
            expired_labels = np.zeros((0,1))
         

        # Add time dimension to the coordinates
        new_coords = add_time_dimension(new_coords,t=2)
        valid_coords = add_time_dimension(valid_coords,t=1)
        expired_coords = add_time_dimension(expired_coords,t=0)

        #print("new coords:",new_coords)
        #print("valid coords:",valid_coords)
        #print("expired coords",expired_coords)

        # Stack everything together
        coords = np.vstack((new_coords, valid_coords, expired_coords))
        feats = np.vstack((new_feats, valid_feats, expired_feats))
        labels = np.vstack((new_labels, valid_labels, expired_labels))

        #print('Datapoint:', coords.shape, feats.shape, labels.shape)  
        return (coords, feats, labels)




class TD3Dataset(Dataset):
    """
    Generate a (coord, feat, label) triplet
    Output:
        coord: A Nx4 tensor with dimensions x,y,z,t
               The full pointcloud resides at dimension t=0
               New points are assigned to the t=1 dimension
        feat: A Nx3 tensor containing R,G,and B channels
        label: A Nx1 tensor containing 1 for valid points and 0 for expired points
    """
    def __init__(self, directory, voxel_size):
        """
        """
        self.directory = directory
        self.voxel_size = voxel_size

    def __len__(self):
        """Return the dataset length"""
        i = 0
        while True:
            path, _, _ = self._get_filepaths(i)
            if not os.path.exists(path):
                break
            i+=1
        if i==0:
            raise ValueError("Could not find dataset in: {}".format(self.directory))
        return i 


    def __getitem__(self, idx):
        """Return a (coord, feat, label) tuple"""
        new_pcd, valid_pcd, expired_pcd = self._get_pointclouds(idx)

        # Get the frame of new points at time t
        new_points = np.array(new_pcd.points)
        new_feats = np.array(new_pcd.colors)
        new_coords, _, new_feats = quantize_points(new_points, new_feats, self.voxel_size)
        new_labels = np.ones((new_points.shape[0], 1))

        # Get the area that we are going to train on
        min_bound = np.min(new_points)
        max_bound = np.max(new_points)

        # Get the valid points at time t
        valid_points, valid_feats = crop_points(valid_pcd, min_bound, max_bound)
        valid_coords, _, valid_feats = quantize_points(valid_points, valid_feats, self.voxel_size)
        valid_labels = np.ones((valid_points.shape[0], 0))

        # Get the expired points at time t
        expired_points, expired_feats = crop_points(expired_pcd, min_bound, max_bound)
        expired_coords, _, expired_feats = quantize_points(expired_points, expired_feats, self.voxel_size)
        expired_labels = np.zeros((expired_points.shape[0], 0))

        # Add time dimension to the coordinates
        new_coords = add_time_dimension(new_coords,t=1)
        valid_coords = add_time_dimension(valid_coords,t=0)
        expired_coords = add_time_dimension(expired_coords,t=0)

        # Stack everything together
        coords = np.vstack((new_coords, valid_coords, expired_coords))
        feats = np.vstack((new_feats, valid_feats, expired_feats))
        labels = np.vstack((new_labels, valid_labels, expired_labels))

        return (coords, feats, labels)


    def _get_filepaths(self,i):
        """Return dataset filepath"""
        current = os.path.join(self.directory, "current-{0}.pcd".format(i))
        valid = os.path.join(self.directory, "valid-{0}.pcd".format(i))
        expired = os.path.join(self.directory, "expired-{0}.pcd".format(i))
        return current, valid, expired


    def _get_pointclouds(self,i):
        """Return the PCD files for the current, valid and expired pointclouds"""
        pointclouds = []
        current, valid, expired = self._get_filepaths(i)
        if not os.path.exists(valid):
            raise IndexError("Data point with path {} does not exist".format(valid))
        for filepath in [current, valid, expired]:
            if os.path.exists(filepath):
                pcd = o3d.io.read_point_cloud(filepath)
            else:
                pcd = o3d.PointCloud()
            pointclouds.append(pcd)
        return pointclouds



class TurtlebotDataset(Dataset):
    """
    Generate a (coord, feat, label) triplet
    Output:
        coord: A Nx4 tensor with dimensions x,y,z,t
               The full pointcloud resides at dimension t=0
               New points are assigned to the t=1 dimension
        feat: A Nx3 tensor containing R,G,and B channels
        label: A Nx1 tensor containing 1 for valid points and 0 for expired points
    """
    def __init__(self, directory, voxel_size):
        """
        """
        self.directory = directory
        self.voxel_size = voxel_size

    def __len__(self):
        """Return the dataset length"""
        i = 0
        while True:
            path, _, _ = self._get_filepaths(i)
            if not os.path.exists(path):
                break
            i+=1
        if i==0:
            raise ValueError("Could not find dataset in: {}".format(self.directory))
        return i 


    def __getitem__(self, idx):
        """Return a (coord, feat, label) tuple"""
        new_pcd, valid_pcd, expired_pcd = self._get_pointclouds(idx)

        # Get the frame of new points at time t
        new_points = np.array(new_pcd.points)
        new_feats = np.array(new_pcd.colors)
        new_coords, _, new_feats = quantize_points(new_points, new_feats, self.voxel_size)
        new_labels = np.ones((new_points.shape[0], 1))

        # Get the area that we are going to train on
        min_bound = np.min(new_points)
        max_bound = np.max(new_points)

        # Get the valid points at time t
        valid_points, valid_feats = crop_points(valid_pcd, min_bound, max_bound)
        valid_coords, _, valid_feats = quantize_points(valid_points, valid_feats, self.voxel_size)
        valid_labels = np.ones((valid_points.shape[0], 0))

        # Get the expired points at time t
        expired_points, expired_feats = crop_points(expired_pcd, min_bound, max_bound)
        expired_coords, _, expired_feats = quantize_points(expired_points, expired_feats, self.voxel_size)
        expired_labels = np.zeros((expired_points.shape[0], 0))

        # Add time dimension to the coordinates
        new_coords = add_time_dimension(new_coords,t=1)
        valid_coords = add_time_dimension(valid_coords,t=0)
        expired_coords = add_time_dimension(expired_coords,t=0)

        # Stack everything together
        coords = np.vstack((new_coords, valid_coords, expired_coords))
        feats = np.vstack((new_feats, valid_feats, expired_feats))
        labels = np.vstack((new_labels, valid_labels, expired_labels))

        return (coords, feats, labels)


    def _get_filepaths(self,i):
        """Return dataset filepath"""
        current = os.path.join(self.directory, "current-{0}.pcd".format(i))
        valid = os.path.join(self.directory, "valid-{0}.pcd".format(i))
        expired = os.path.join(self.directory, "expired-{0}.pcd".format(i))
        return current, valid, expired


    def _get_pointclouds(self,i):
        """Return the PCD files for the current, valid and expired pointclouds"""
        pointclouds = []
        current, valid, expired = self._get_filepaths(i)
        if not os.path.exists(valid):
            raise IndexError("Data point with path {} does not exist".format(valid))
        for filepath in [current, valid, expired]:
            if os.path.exists(filepath):
                pcd = o3d.io.read_point_cloud(filepath)
            else:
                pcd = o3d.PointCloud()
            pointclouds.append(pcd)
        return pointclouds

