import numpy as np
from open3d import *
from tqdm import tqdm

RED = [1,0,0]


def set_color(pcd, index, color):
    """Set points[index] to color"""
    np.asarray(pcd.colors)[index, :] = color


def union(pcd1, pcd2):
    """Take the union of two clouds"""
    pcd = PointCloud()
    pcd.points = Vector3dVector(np.vstack((pcd1.points, pcd2.points)))
    pcd.colors = Vector3dVector(np.vstack((pcd1.colors, pcd2.colors)))
    return pcd


def get_new_points(before, after):
    """Return new points as a pointcloud"""
    # Downsample large pointcloud
    print("Downsample the point cloud with a voxel of 0.05")
    before = voxel_down_sample(before, voxel_size = 0.05)
    pcd_tree = KDTreeFlann(before)
    indices = []
    new_points = 0

    print("Searching for new points... ", end="")
    for i,point in tqdm(enumerate(after.points)):
        [k, idx, _] = pcd_tree.search_hybrid_vector_3d(point, 0.05, 2)
        if k<1:
            indices.append(i)
            new_points += 1
    print("Done")
    print("Found %i new points"%new_points)

    new_points = PointCloud()
    new_points.points = Vector3dVector(np.asarray(after.points)[indices,:])
    new_points.colors = Vector3dVector(np.asarray(after.colors)[indices,:])
    return new_points


def diff(before, after):
    """
    Diff the pointclouds in two files
    Returns the union of the two pointclouds, where new points are red.
    """
    # Downsample large pointcloud
    print("Downsample the point cloud with a voxel of 0.05")
    before = voxel_down_sample(before, voxel_size = 0.05)
    pcd_tree = KDTreeFlann(before)
    new_points = 0

    print("Searching for new points... ", end="")
    for i,point in tqdm(enumerate(after.points)):
        [k, idx, _] = pcd_tree.search_hybrid_vector_3d(point, 0.05, 2)
        if k<1:
            new_points += 1
            set_color(after, i, RED)
    print("Done")
    print("Found %i new points"%new_points)

    return union(before,after)


if __name__ == "__main__":
    pass