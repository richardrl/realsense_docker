# creates object pointcloud from multiple cameras
# saves pointcloud to disk
# NOTE: assumes single object on table

import glob
from pathlib import Path
import numpy as np
from camera import Camera
from utils.camera_util import convert_depth_to_pointcloud
from utils import visualization_util
import open3d
import datetime
import sys
import torch
from scipy.spatial.transform import Rotation as R

"""
Start Config
"""
cam_color_segment = False
num_cameras = 4
filter_height = -.125
filter_workspace = True

table_rotation_x = .05

# workspace limits in world frame
workspace_limits = np.asarray([[0.6384-.25, 0.6384+.25], [.1325-.35, .1325+.35], [filter_height, filter_height+.2]])
# todo: make workspace limits as a cube
# todo: cropout points belonging to arm using urdf
# can do the above by loading urdf into pybullet, then using point checks... a lot of work mb

"""
End Config
"""

if len(sys.argv) == 1:
    object_name = None
else:
    object_name = sys.argv[1]

# load extrinsics

serial_no2intrinsics_dic = dict()
serial_no2extrinsics_dic = dict()
serial_no2depth_imgs_dic = dict()
serial_no2color_imgs_dic = dict()

txts = [f for f in glob.glob(str(Path("../out/") / "*.txt"), recursive=True)]

assert txts

cameras = [Camera(port=50000+i) for i in range(num_cameras)]

for txt in txts:
    if "camera_pose" in txt:
        X_WC = np.loadtxt(txt)
        serial_no2extrinsics_dic[txt.split("_")[0].split("/")[-1]] = X_WC.copy()

# convert the depth images from each camera into pointclouds

for cam in cameras:
    color_img, depth_img = cam.get_data()

    # cam.serial_number = 0
    serial_no2depth_imgs_dic[cam.serial_number] = depth_img
    serial_no2intrinsics_dic[cam.serial_number] = cam.intrinsics
    serial_no2color_imgs_dic[cam.serial_number] = color_img

geometries = []
colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, 0]]

# convert depth to camera frame pointcloud
# merge all world frame pointclouds
import matplotlib.pyplot as plt

# fig = plt.figure()

aggregate_pc_lst = []
aggregate_rgb_lst = []
for cam_idx, serial_no in enumerate(serial_no2depth_imgs_dic.keys()):
    # if cam_idx != 3:
    #     continue

    # debugging alignment
    # plt.figure(figsize=(10, 10))
    #
    # plt.imshow(serial_no2color_imgs_dic[serial_no])
    #
    # plt.imshow(serial_no2depth_imgs_dic[serial_no],cmap='nipy_spectral_r',alpha=.5)
    #
    # plt.show()

    np.set_printoptions(formatter={'float': "{0:0.3f}".format})
    print(f"{serial_no}")

    print(f"Extrinsics: ")
    print(serial_no2extrinsics_dic[serial_no])

    p_WorldScene, p_CamScene = convert_depth_to_pointcloud(serial_no2depth_imgs_dic[serial_no],
                                               serial_no2extrinsics_dic[serial_no],
                                               serial_no2intrinsics_dic[serial_no])

    # debug individual p_CamScene
    # open3d.visualization.draw_geometries([visualization_util.make_point_cloud_o3d(p_CamScene[p_CamScene[:, 2] < 1],
    #                                                            # serial_no2color_imgs_dic[serial_no].reshape(-1, 3)[p_CamScene[:, 2] < 1],
    #                                                                               [0, 0, 0],
    #                                                            normalize_color=True),
    #                                       open3d.geometry.TriangleMesh.create_coordinate_frame(.03, [0, 0, 0])])

    if cam_color_segment:
        geometries.append(visualization_util.make_point_cloud_o3d(p_WorldScene[p_CamScene[:, 2] < 1],
                                                                   colors[cam_idx],
                                                                   normalize_color=False))
    else:
        color_ims = serial_no2color_imgs_dic[serial_no]
        img_height, img_width, _ = color_ims.shape

        uv_coords = np.mgrid[0: img_height,
                    0: img_width].reshape(2, -1)

        uv_coords[[0, 1], :] = uv_coords[[1, 0], :]

        # since uv_coords has been changed to be X, Y, and color_ims is ordered Y, X, we index this way
        reshaped_color = color_ims[uv_coords[1, :], uv_coords[0, :]]

        # filter by world scene height
        # and filter by camscene depth

        p_WorldScene_cropped = p_WorldScene[(p_CamScene[:, 2] < 1) *
                                                                               (p_WorldScene[:, 0] > workspace_limits[0][0]) *
                                                                               (p_WorldScene[:, 0] < workspace_limits[0][1]) *
                                                                               (p_WorldScene[:, 1] > workspace_limits[1][0]) *
                                                                               (p_WorldScene[:, 1] < workspace_limits[1][1])]
                                                                               # (p_WorldScene[:, 2] > workspace_limits[2][0]) *
                                                                               # (p_WorldScene[:, 2] < workspace_limits[2][1])]
        rgb_cropped = reshaped_color[(p_CamScene[:, 2] < 1) *
                                                                                  (p_WorldScene[:, 0] >
                                                                                   workspace_limits[0][0]) *
                                                                                  (p_WorldScene[:, 0] <
                                                                                   workspace_limits[0][1]) *
                                                                                  (p_WorldScene[:, 1] >
                                                                                   workspace_limits[1][0]) *
                                                                                  (p_WorldScene[:, 1] <
                                                                                   workspace_limits[1][1])]
                                                                                  # (p_WorldScene[:, 2] >
                                                                                  #  workspace_limits[2][0]) *
                                                                                  # (p_WorldScene[:, 2] <
                                                                                  #  workspace_limits[2][1])]
        aggregate_pc_lst.append(p_WorldScene_cropped)
        aggregate_rgb_lst.append(rgb_cropped)
        # geometries.append(visualization_util.make_point_cloud_o3d(p_WorldScene_cropped,
        #                                                            rgb_cropped,
        #                                                            normalize_color=True))

aggregate_pc = np.concatenate(aggregate_pc_lst)
aggregate_rgb = np.concatenate(aggregate_rgb_lst)

aggregate_pcd = visualization_util.make_point_cloud_o3d(aggregate_pc,
                                        aggregate_rgb,
                                                        normalize_color=True)

# remove table points
table_normal = np.array([0, 0, 1])

table_height_vector = table_normal * filter_height

# rotate about x axis
new_normal = R.from_euler("x", table_rotation_x).apply(table_normal)

# filter out table points

mask = np.logical_and((np.array(aggregate_pcd.points) @ new_normal) > workspace_limits[2][0],
                       (np.array(aggregate_pcd.points) @ new_normal) < workspace_limits[2][1]).copy()
aggregate_pcd.points = open3d.utility.Vector3dVector(np.array(aggregate_pcd.points)[mask, :])
aggregate_pcd.colors = open3d.utility.Vector3dVector(np.array(aggregate_pcd.colors)[mask, :])

# remove noise
aggregate_pcd, idxs = aggregate_pcd.remove_radius_outlier(nb_points=50, radius=0.01)

if object_name is not None:
    d = datetime.datetime.now()
    filename = f"{d:%m-%d-%Y_%H:%M:%S}_{object_name}.torch".format(d=d, object_name=object_name)

    save_dic = dict(points=np.array(aggregate_pcd.points),
                    colors=np.array(aggregate_pcd.colors),
                    object_name=object_name)


# table_box = open3d.geometry.TriangleMesh.create_box(width=workspace_limits[0][1] - workspace_limits[0][0],
#                                                     height=workspace_limits[1][1]-workspace_limits[1][0],
#                                                     depth=.003)
# table_box.paint_uniform_color([0, 0, 1])
#
# table_box.translate(-table_box.get_center())
#
# table_box.translate([(workspace_limits[0][0] + workspace_limits[0][1]) / 2,
#                      (workspace_limits[1][0] + workspace_limits[1][1]) / 2,
#                      workspace_limits[2][0]])

print(np.array(aggregate_pcd.points).mean(axis=0))

obb = open3d.geometry.OrientedBoundingBox()
obb = obb.create_from_points(aggregate_pcd.points)

open3d.visualization.draw_geometries([aggregate_pcd,
                                      open3d.geometry.TriangleMesh.create_coordinate_frame(.03, [0, 0, 0]),
                                      # table_box,
                                      obb])

torch.save(save_dic, f"../out/samples/{filename}")
