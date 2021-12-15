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
cam_color_segment = True

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

cameras = [Camera(port=50000 + i) for i in range(3)]

for txt in txts:
    if "camera_pose" in txt:
        X_WC = np.loadtxt(txt)
        serial_no2extrinsics_dic[txt.split("_")[0].split("/")[-1]] = X_WC.copy()

# convert the depth images from each camera into pointclouds

for cam in cameras:
    color_img, depth_img = cam.get_data()
    serial_no2depth_imgs_dic[cam.serial_number] = depth_img
    serial_no2intrinsics_dic[cam.serial_number] = cam.intrinsics
    serial_no2color_imgs_dic[cam.serial_number] = color_img

geometries = []
colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

# convert depth to camera frame pointcloud
# merge all world frame pointclouds
import matplotlib.pyplot as plt

for cam_idx, serial_no in enumerate(serial_no2depth_imgs_dic.keys()):
    # plt.imshow(serial_no2color_imgs_dic[serial_no])
    # plt.show()
    # plt.imshow(serial_no2depth_imgs_dic[serial_no],cmap='nipy_spectral_r')
    # plt.show()
    #
    np.set_printoptions(formatter={'float': "{0:0.3f}".format})
    print(f"{serial_no}")

    print(f"Extrinsics: ")
    print(serial_no2extrinsics_dic[serial_no])

    p_WorldScene, p_CamScene = convert_depth_to_pointcloud(serial_no2depth_imgs_dic[serial_no],
                                               serial_no2extrinsics_dic[serial_no],
                                               serial_no2intrinsics_dic[serial_no])

    # debug individual p_CamScene
    # open3d.visualization.draw_geometries([visualization_util.make_point_cloud_o3d(p_CamScene[p_CamScene[:, 2] < 1],
    #                                                            serial_no2color_imgs_dic[serial_no].reshape(-1, 3)[p_CamScene[:, 2] < 1],
    #                                                            normalize_color=True),
    #                                       open3d.geometry.TriangleMesh.create_coordinate_frame(.03, [0, 0, 0])])

    if cam_color_segment:
        geometries.append(visualization_util.make_point_cloud_o3d(p_WorldScene[p_CamScene[:, 2] < 1],
                                                                   colors[cam_idx],
                                                                   normalize_color=False))
    else:
        geometries.append(visualization_util.make_point_cloud_o3d(p_WorldScene[p_CamScene[:, 2] < 1],
                                                                   serial_no2color_imgs_dic[serial_no].reshape(-1, 3)[p_CamScene[:, 2] < 1],
                                                                   normalize_color=True))

open3d.visualization.draw_geometries(geometries)