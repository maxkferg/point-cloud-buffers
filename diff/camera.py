# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

# examples/Python/Tutorial/Advanced/headless_rendering.py

import os
import json
from open3d import *
import numpy as np
import matplotlib.pyplot as plt

IMAGE_PATH = "diff/output/image/"
DEPTH_PATH = "diff/output/depth/"
OPTIONS = "diff/options.json"

HEIGHT = 1080
WIDTH = 1920

def draw_geometry_with_camera_trajectory(geometry, trajectory_file):
    print("Reading trajectory from %s"%trajectory_file)
    draw_geometry_with_camera_trajectory.index = -1
    draw_geometry_with_camera_trajectory.trajectory = read_pinhole_camera_trajectory(trajectory_file)
    draw_geometry_with_camera_trajectory.vis = Visualizer()
    if not os.path.exists(IMAGE_PATH):
        os.makedirs(IMAGE_PATH)
    if not os.path.exists(DEPTH_PATH):
        os.makedirs(DEPTH_PATH)
    def move_forward(vis):
        # This function is called within the Visualizer::run() loop
        # The run loop calls the function, then re-render
        # So the sequence in this function is to:
        # 1. Capture frame
        # 2. index++, check ending criteria
        # 3. Set camera
        # 4. (Re-render)
        ctr = vis.get_view_control()
        glb = draw_geometry_with_camera_trajectory
        if glb.index >= 0:
            print("Capture image {:05d}".format(glb.index))
            depth = vis.capture_depth_float_buffer(False)
            image = vis.capture_screen_float_buffer(False)
            plt.imsave(DEPTH_PATH+"{:05d}.png".format(glb.index), np.asarray(depth), dpi = 1)
            plt.imsave(IMAGE_PATH+"{:05d}.png".format(glb.index), np.asarray(image), dpi = 1)
            #vis.capture_depth_image("depth/{:05d}.png".format(glb.index), False)
            #vis.capture_screen_image("image/{:05d}.png".format(glb.index), False)
        glb.index = glb.index + 1
        if glb.index < len(glb.trajectory.extrinsic):
            #camera = glb.trajectory["parameters"][glb.index]
            #intrinsic = PinholeCameraIntrinsic(1920, 1080, 525, 525, 320, 240)
            #extrinsic = np.array(camera["extrinsic"]).reshape((4,4))
            intrinsic = PinholeCameraIntrinsic()
            cx = WIDTH/2.0-0.5
            cy = HEIGHT/2.0-0.5
            intrinsic.set_intrinsics(WIDTH, HEIGHT, 935.3, 935.3, cx, cy)
            #intrinsic.intrinsic_matrix = intrinsic.intrinsic_matrix * np.array([[1,0,0],[0,1,0],[0,0,-1]])
            extrinsic = glb.trajectory.extrinsic[glb.index]
            #extrinsic = extrinsic*np.eye(4)
            ctr.convert_from_pinhole_camera_parameters(intrinsic, extrinsic)
        else:
            draw_geometry_with_camera_trajectory.vis.register_animation_callback(None)
        return False
    print("Drawing geometry:",geometry)
    vis = draw_geometry_with_camera_trajectory.vis
    vis.create_window(width=WIDTH, height=HEIGHT)
    [vis.add_geometry(g) for g in geometry]
    vis.get_render_option().load_from_json(OPTIONS)
    vis.register_animation_callback(move_forward)
    vis.run()
    vis.destroy_window()