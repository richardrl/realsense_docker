import open3d as o3d
import numpy as np


def make_point_cloud_o3d(points, color, normalize_color=False):
    """

    Args:
        points: num_points x 3
        color: num_points x 3
            COLOR MUST BE BETWEEN 0 and 1!!!!

    Returns:

    """
    # if isinstance(points, torch.Tensor):
    #     points = points.data.cpu().numpy()

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.copy())

    if normalize_color:
        color = np.array(color)/255
    else:
        color = np.array(color)

    if len(color.shape) == 1:
        pcd.colors = o3d.utility.Vector3dVector(np.tile(color, (points.shape[0], 1)))
    else:
        pcd.colors = o3d.utility.Vector3dVector(color.copy())
    return pcd