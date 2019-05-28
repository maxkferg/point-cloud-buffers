"""
Run several visual tests for debugging the pipeline
"""
import open3d as o3d
from simulator.pointstream import SceneGenerator
from simulator.buffer import PointBuffer


def render_frames(point_buffer):
    """Render frames from the buffer"""
    for i in range(10):
        points, colors, classes = point_buffer.get_frame()
        pcd = o3d.PointCloud()
        pcd.points = o3d.Vector3dVector(points)
        pcd.colors = o3d.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pcd])


def render_full_cloud(point_buffer):
    """Render the full cloud as it is constructed"""
    for i in range(10):
        point_buffer.get_frame()
        pcd = point_buffer.get_buffer_points()
        o3d.visualization.draw_geometries([pcd])


def render_with_expired(point_buffer):
    """Render the full cloud with expired point shown in red"""
    for i in range(10):
        point_buffer.get_frame()
        valid = point_buffer.get_valid_points()
        invalid = point_buffer.get_expired_points()
        invalid.paint_uniform_color([1, 0.1, 0.1])
        if i>4:
            o3d.visualization.draw_geometries([valid, invalid])


if __name__=="__main__":
    metadata_file = "simulator/dataset/metadata.csv"
    point_stream = SceneGenerator(metadata_file)
    point_buffer = PointBuffer(point_stream)
    #render_frames(point_buffer)
    #render_full_cloud(point_buffer)
    render_with_expired(point_buffer)