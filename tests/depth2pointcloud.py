import glob
from pathlib import Path
import numpy as np
from real.camera import Camera
from utils.camera_util import convert_depth_to_pointcloud
from utils import visualization_util
import open3d

"""
Start Config
"""
cam_color_segment = False
num_cameras = 4
filter_height = -.125
filter_workspace = True

# workspace limits in world frame
workspace_limits = np.asarray([[0.6384-.25, 0.6384+.25], [.1325-.25, .1325+.25], [-.125, -.125+.2]])
# todo: make workspace limits as a cube
# todo: cropout points belonging to arm using urdf
# can do the above by loading urdf into pybullet, then using point checks... a lot of work mb

"""
End Config
"""

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
colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

# convert depth to camera frame pointcloud
# merge all world frame pointclouds
import matplotlib.pyplot as plt

# fig = plt.figure()

aggregate_pc_lst = []
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

    aggregate_pc_lst.append(p_WorldScene)
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

        geometries.append(visualization_util.make_point_cloud_o3d(p_WorldScene[(p_CamScene[:, 2] < 1) *
                                                                               (p_WorldScene[:, 0] > workspace_limits[0][0]) *
                                                                               (p_WorldScene[:, 0] < workspace_limits[0][1]) *
                                                                               (p_WorldScene[:, 1] > workspace_limits[1][0]) *
                                                                               (p_WorldScene[:, 1] < workspace_limits[1][1]) *
                                                                               (p_WorldScene[:, 2] > workspace_limits[2][0]) *
                                                                               (p_WorldScene[:, 2] < workspace_limits[2][1])],
                                                                   reshaped_color[(p_CamScene[:, 2] < 1) *
                                                                                  (p_WorldScene[:, 0] >
                                                                                   workspace_limits[0][0]) *
                                                                                  (p_WorldScene[:, 0] <
                                                                                   workspace_limits[0][1]) *
                                                                                  (p_WorldScene[:, 1] >
                                                                                   workspace_limits[1][0]) *
                                                                                  (p_WorldScene[:, 1] <
                                                                                   workspace_limits[1][1]) *
                                                                                  (p_WorldScene[:, 2] >
                                                                                   workspace_limits[2][0]) *
                                                                                  (p_WorldScene[:, 2] <
                                                                                   workspace_limits[2][1])],
                                                                   normalize_color=True))

aggregate_pc = np.concatenate(aggregate_pc_lst)
import torch
torch.save(aggregate_pc, "../out/aggregate_pc.torch")
open3d.visualization.draw_geometries(geometries)