import numpy as np
import open3d as o3d 
from scipy.misc import imsave
from datasets import TD3Dataset
from moviepy.video.VideoClip import ImageClip
from moviepy.editor import concatenate_videoclips


def current_pcd_to_video(dataset):
    clips = []
    filenames = []
    for i in range(len(dataset)):
        new_pcd, _, _ = dataset._get_pointclouds(i)
        img = draw_geometry_to_image([new_pcd])
        filename = "videos/tmp/tmp-{}.png".format(i)
        filenames.append(filename)
        imsave(filename, img)
        clip = ImageClip(filename).set_duration(2)
        clips.append(clip)
    concat_clip = concatenate_videoclips(clips, method="compose")
    concat_clip.write_videofile("videos/current_pcd.mp4", fps=24)


def valid_pcd_to_video(dataset):
    clips = []
    filenames = []
    for i in range(len(dataset)):
        _, valid_pcd, _ = dataset._get_pointclouds(i)
        img = draw_geometry_to_image([valid_pcd])
        filename = "videos/tmp/tmp-{}.png".format(i)
        filenames.append(filename)
        imsave(filename, img)
        clip = ImageClip(filename).set_duration(0.2)
        clips.append(clip)
    concat_clip = concatenate_videoclips(clips, method="compose")
    concat_clip.write_videofile("videos/valid_pcd.mp4", fps=24)


def expired_pcd_to_video(dataset):
    clips = []
    filenames = []
    for i in range(len(dataset)):
        _, _, expired_pcd = dataset._get_pointclouds(i)
        img = draw_geometry_to_image([expired_pcd])
        filename = "videos/tmp/tmp-{}.png".format(i)
        filenames.append(filename)
        imsave(filename, img)
        clip = ImageClip(filename).set_duration(0.2)
        clips.append(clip)
    concat_clip = concatenate_videoclips(clips, method="compose")
    concat_clip.write_videofile("videos/expired_pcd.mp4", fps=24)


def classification_pcd_to_video(dataset):
    clips = []
    filenames = []
    for i in range(len(dataset)):
        new_pcd, valid_pcd, expired_pcd = dataset._get_pointclouds(i)
        # Color new_pcd green. Old PCD red
        new_colors = np.array([0.3,3,0.3])*np.array(new_pcd.colors)
        expired_colors = np.array([3,0.3,0.3])*np.array(new_pcd.colors)
        new_pcd.colors = o3d.Vector3dVector(new_colors)
        expired_pcd.colors = o3d.Vector3dVector(expired_colors)
        img = draw_geometry_to_image([new_pcd, valid_pcd, expired_pcd])
        filename = "videos/tmp/tmp-{}.png".format(i)
        filenames.append(filename)
        imsave(filename, img)
        clip = ImageClip(filename).set_duration(0.2)
        clips.append(clip)
    concat_clip = concatenate_videoclips(clips, method="compose")
    concat_clip.write_videofile("videos/expired_pcd.mp4", fps=24)


def video_before(dataset, stop=10):
    clips = []
    filenames = []
    for i in range(len(dataset)):
        new_pcd, valid_pcd, expired_pcd = dataset._get_pointclouds(i)
        # Swap from rgb to bgr
        new_colors = np.array(new_pcd.colors)[:, ::-1]
        valid_colors = np.array(valid_pcd.colors)[:, ::-1]
        expired_colors = np.array(expired_pcd.colors)[:, ::-1]

        # Color new_pcd green. Old PCD red
        new_colors = np.array([0.3,3,0.3])*np.array(new_colors)
        # Only show the expired points at the end
        if i>(len(dataset)-10) or i>(stop-10):
            expired_colors = np.array([3,0.1,0.1])*np.array(expired_colors)
        new_pcd.colors = o3d.Vector3dVector(new_colors)
        valid_pcd.colors = o3d.Vector3dVector(valid_colors)
        expired_pcd.colors = o3d.Vector3dVector(expired_colors)
        img = draw_geometry_to_image([new_pcd, valid_pcd, expired_pcd])
        filename = "videos/tmp/tmp-{}.png".format(i)
        filenames.append(filename)
        imsave(filename, img)
        clip = ImageClip(filename).set_duration(0.2)
        clips.append(clip)
        if i==stop: break
    concat_clip = concatenate_videoclips(clips, method="compose")
    concat_clip.write_videofile("videos/video_before.mp4", fps=24)


def video_expiring(dataset, stop=10):
    clips = []
    filenames = []
    for i in range(len(dataset)):
        new_pcd, valid_pcd, expired_pcd = dataset._get_pointclouds(i)
        # Swap from rgb to bgr
        new_colors = np.array(new_pcd.colors)[:, ::-1]
        valid_colors = np.array(valid_pcd.colors)[:, ::-1]
        expired_colors = np.array(expired_pcd.colors)[:, ::-1]

        # Color new_pcd green. Old PCD red
        new_colors = np.array([0.3,3,0.3])*np.array(new_colors)
        expired_colors = np.array([3,0.1,0.1])*np.array(expired_colors)
        new_pcd.colors = o3d.Vector3dVector(new_colors)
        valid_pcd.colors = o3d.Vector3dVector(valid_colors)
        expired_pcd.colors = o3d.Vector3dVector(expired_colors)
        img = draw_geometry_to_image([new_pcd, valid_pcd, expired_pcd])
        filename = "videos/tmp/tmp-{}.png".format(i)
        filenames.append(filename)
        imsave(filename, img)
        clip = ImageClip(filename).set_duration(0.2)
        clips.append(clip)
        if i==stop: break
    concat_clip = concatenate_videoclips(clips, method="compose")
    concat_clip.write_videofile("videos/video_expiring.mp4", fps=24)


def video_removing(dataset, stop=10):
    clips = []
    filenames = []
    for i in range(len(dataset)):
        new_pcd, valid_pcd, expired_pcd = dataset._get_pointclouds(i)
        # Swap from rgb to bgr
        new_colors = np.array(new_pcd.colors)[:, ::-1]
        valid_colors = np.array(valid_pcd.colors)[:, ::-1]
        expired_colors = np.array(expired_pcd.colors)[:, ::-1]

        # Color new_pcd green. Old PCD red
        new_colors = np.array([0.3,3,0.3])*np.array(new_colors)
        expired_colors = np.array([3,0.1,0.1])*np.array(expired_colors)
        new_pcd.colors = o3d.Vector3dVector(new_colors)
        valid_pcd.colors = o3d.Vector3dVector(valid_colors)
        expired_pcd.colors = o3d.Vector3dVector(expired_colors)
        # Crop everything but the last 1000 expired points
        expired_pcd.colors = o3d.Vector3dVector(np.array(expired_pcd.colors)[-10000:,:])
        expired_pcd.points = o3d.Vector3dVector(np.array(expired_pcd.points)[-10000:,:])
        if i<(stop-10):
            img = draw_geometry_to_image([new_pcd, valid_pcd, expired_pcd])
        else:
            img = draw_geometry_to_image([new_pcd, valid_pcd])
        filename = "videos/tmp/tmp-{}.png".format(i)
        filenames.append(filename)
        imsave(filename, img)
        clip = ImageClip(filename).set_duration(0.2)
        clips.append(clip)
        if i==stop: break
    concat_clip = concatenate_videoclips(clips, method="compose")
    concat_clip.write_videofile("videos/video_removing.mp4", fps=24)





def draw_geometry_to_image(pcd, fov_step=30):
    # Create visualizer
    vis = o3d.Visualizer()
    vis.create_window()
    for geom in pcd:
        vis.add_geometry(geom)
    vis.poll_events()
    # Set the render options and viewpoint 
    vis.get_render_option().load_from_json("assets/render_options.json")
    param = o3d.read_pinhole_camera_parameters(filename="assets/viewpoint.json")
    ctr = vis.get_view_control()
    ctr.convert_from_pinhole_camera_parameters(param)
    vis.update_geometry()
    vis.poll_events()
    vis.update_renderer()    
    # vis.run()
    # Capture image
    image = vis.capture_screen_float_buffer()
    vis.destroy_window()
    return image


def record_view_point(dataset):
    size = len(dataset)
    _, valid_pcd, _ = dataset._get_pointclouds(size-1)
    save_view_point(valid_pcd, "assets/viewpoint.json")


def save_view_point(pcd, filename):
    vis = o3d.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run() # user changes the view and press "q" to terminate
    param = vis.get_view_control().convert_to_pinhole_camera_parameters()
    o3d.write_pinhole_camera_parameters(filename, param)
    vis.destroy_window()
    print("Saved viewpoint to:",filename)


def load_view_point(pcd, filename):
    vis = o3d.Visualizer()
    vis.create_window()
    ctr = vis.get_view_control()
    param = o3d.read_pinhole_camera_parameters(filename)
    vis.add_geometry(pcd)
    ctr.convert_from_pinhole_camera_parameters(param)
    vis.run()
    vis.destroy_window()



def draw_geometry_with_key_callback(pcd):

    def change_background_to_black(vis):
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0, 0, 0])
        return False

    def load_render_option(vis):
        vis.get_render_option().load_from_json("../../TestData/renderoption.json")
        return False

    def capture_depth(vis):
        depth = vis.capture_depth_float_buffer()
        plt.imshow(np.asarray(depth))
        plt.show()
        return False

    def capture_image(vis):
        image = vis.capture_screen_float_buffer()
        plt.imshow(np.asarray(image))
        plt.show()
        return False

    key_to_callback = {}
    key_to_callback[ord("K")] = change_background_to_black
    key_to_callback[ord("R")] = load_render_option
    key_to_callback[ord(",")] = capture_depth
    key_to_callback[ord(".")] = capture_image
    o3d.visualization.draw_geometries_with_key_callbacks([pcd], key_to_callback)


if __name__=="__main__":
    directory = "simulator/dataset/points2"
    dataset = TD3Dataset(directory, voxel_size=0.05)
    #record_view_point(dataset)
    #valid_pcd_to_video(dataset)
    #expired_pcd_to_video(dataset)
    #classification_pcd_to_video(dataset)
    video_before(dataset, stop=50)
    #video_expiring(dataset, stop=50)
    #video_removing(dataset, stop=50)
    