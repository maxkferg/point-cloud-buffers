"""
Define training datasets for point age task
"""
import numpy as np
import open3d as o3d
from torch.utils.data import Dataset
from simulator.pointstream import SceneGenerator
from simulator.buffer import PointBuffer
import MinkowskiEngine as ME

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
