import os
import json
import scipy
import numpy as np
import matplotlib.pyplot as plt
from open3d import *
from sklearn.cluster import MeanShift
from diff.diff import diff, get_new_points
from diff.simulate import SceneGenerator
from diff.camera import draw_geometry_with_camera_trajectory 
from rendering.sim.simulation.environment.simulation_env import SimRobotEnv

GENERATE_ASSETS = False
METADATA = "vision/assets/dataset/stage01/metadata.csv"
FOLDER = "diff/assets"

rgb_filename_before = os.path.join(FOLDER, "rgb/before-%i.png")
rgb_filename_after = os.path.join(FOLDER, "rgb/after-%i.png")
pts_filename_before = os.path.join(FOLDER, "points/before-%i.ply")
pts_filename_after = os.path.join(FOLDER, "points/after-%i.ply")
cloud_filename_before = os.path.join(FOLDER, "cloud/before.ply")
cloud_filename_after = os.path.join(FOLDER, "cloud/after.ply")
camera_filename = os.path.join(FOLDER, "camera/view-%i.json")
trajectory_filename = os.path.join(FOLDER, "camera/trajectory.json")


def load_ply(filename):
    """
    Save points to a file
    Points is a column vector with rows (x,y,z)
    """
    pcd = read_point_cloud(filename)
    points = np.asarray(pcd.points)
    pcd.points = Vector3dVector(points)
    return pcd
        

def save_ply(filename, points, rgb):
    """
    Save points to a file
    Points is a column vector with rows (x,y,z)
    """
    xyz = points[:,:3]
    pcd = PointCloud()
    pcd.points = Vector3dVector(xyz)
    pcd.colors = Vector3dVector(rgb)
    write_point_cloud(filename, pcd)


def save_rgb(filename, rgb):
    plt.imsave(filename, rgb)


def save_camera(filename, camera):
    """Save a camera view to json"""
    with open(filename, 'w') as outfile:  
        json.dump(camera, outfile, indent=4)


def save_trajectory(filename, cameras):
    """Save a camera view to json"""
    data = {
        "class_name" : "PinholeCameraTrajectory",
        "extrinsic": [c["extrinsic"] for c in cameras],
        "intrinsic": cameras[0]["intrinsic"],
        "version_major" : 1,
        "version_minor" : 0
    }
    with open(trajectory_filename, 'w') as outfile:  
        json.dump(data, outfile, indent=4)


def save_full_cloud(filename, scene, n, skip, hop):
    """
    Save a full pointcloud from a simulation
    """
    scene.skip(skip)
    pcd = scene.get_full_pointcloud(n=n, hop=hop)
    write_point_cloud(filename, pcd)


def generate_scene_files(env, n, skip=0, hop=0):
    """
    Generate all of the files that are needed for demonstation tests:
    Generates the following files:
    - Full point cloud: cloud/before.ply
    - Point files with objects: points/after-%i.ply
    - RGB files: rgb/after-%i.png
    - Camera Files: camera/after-%i.ply
    """

    scene = SceneGenerator(env, METADATA)

    # Generate the scans with no objects
    #for i,output in enumerate(scene):
    #    points, rgb = output
    #    error = np.zeros((points.shape[0], 1))
    #    save_ply(pts_filename%i, points)
    #    save_rgb(rgb_filename%i, rgb)
    #    if i>n:
    #        break

    # Generate scan with points
    save_full_cloud(cloud_filename_before, scene, n=n, skip=skip, hop=hop)

    # Generate the scans with objects
    scene.reset(add_objects=True, probability=0.1)
    scene.skip(skip)
    trajectory = []
    for i, output in enumerate(scene):
        points, rgb, camera = output
        width = camera["intrinsic"]["width"]
        height = camera["intrinsic"]["height"]
        save_ply(pts_filename_after%i, points, rgb)
        save_rgb(rgb_filename_after%i, scene.env.render(width,height))
        save_camera(camera_filename%i, camera)
        trajectory.append(camera)
        scene.skip(hop)
        if i>n:
            break
    save_trajectory(trajectory_filename, trajectory)


def color_clusters(pcd):
    points = np.asarray(pcd.points)
    colors = np.zeros_like(points)
    cmap = plt.get_cmap("tab10")

    clustering = MeanShift().fit(points)
    for i, point in enumerate(points):
        cls = clustering.labels_[i]
        colors[i,:3] = cmap(cls)[:3]

    pcd.colors = Vector3dVector(colors)
    return pcd, clustering



def draw_cube_around_objects(pcd,clustering):
    classes = set(clustering.labels_)
    points = np.asarray(pcd.points)
    cmap = plt.get_cmap("tab10")
    boxes = []
    for cls in classes:
        cls_color = cmap(cls)[:3]
        cls_points = points[clustering.labels_==cls,:]
        xmin,ymin,zmin = np.min(cls_points,axis=0)
        xmax,ymax,zmax = np.max(cls_points,axis=0)
        boxes.append(draw_cube(xmin, ymin, zmin, xmax, ymax, zmax, cls_color))
    return boxes



def draw_cube(xmin, ymin, zmin, xmax, ymax, zmax, color=[1,0,0]):
    """Draw a cubic that consists of 8 points and 12 lines"""
    points = [[xmin,ymin,zmin],[xmax,ymin,zmin],[xmin,ymax,zmin],[xmax,ymax,zmin],
              [xmin,ymin,zmax],[xmax,ymin,zmax],[xmin,ymax,zmax],[xmax,ymax,zmax]]
    lines = [[0,1],[0,2],[1,3],[2,3],
             [4,5],[4,6],[5,7],[6,7],
             [0,4],[1,5],[2,6],[3,7]]
    colors = [color for i in range(len(lines))]
    line_set = LineSet()
    line_set.points = Vector3dVector(points)
    line_set.lines = Vector2iVector(lines)
    line_set.colors = Vector3dVector(colors)
    return line_set



def diff_test(before_ply, after_ply):
    #pcd = diff(before_ply, after_ply)
    #draw_geometries([pcd])
    # Get the new points only
    before = load_ply(before_ply)
    after = load_ply(after_ply)

    new_points = get_new_points(before, after)
    new_points, clustering = color_clusters(new_points)
    geometries = draw_cube_around_objects(new_points, clustering)
    draw_geometries([new_points,before]+geometries)



def draw_trajectory(before_ply, after_ply, n):
    # Load the trajectory
    before = load_ply(before_ply)
    after = load_ply(after_ply)

    # Make a scene with the 3rd element rendered
    new_points = get_new_points(before, after)
    new_points, clustering = color_clusters(new_points)
    cubes = draw_cube_around_objects(new_points, clustering)
    geometery = [new_points,before]+cubes

    draw_geometry_with_camera_trajectory(geometery, trajectory_filename)



if __name__=="__main__":
    n = 40
    if GENERATE_ASSETS:
        generate_scene_files(SimRobotEnv, n=n, skip=260, hop=4)

    #diff_test(cloud_filename_before, pts_filename_after%2)
    draw_trajectory(cloud_filename_before, pts_filename_after%6, n)




