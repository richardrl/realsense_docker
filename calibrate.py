#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
from real.camera import Camera
from robot import Robot
from scipy import optimize
from mpl_toolkits.mplot3d import Axes3D
import os
import charuco_util
from scipy.spatial.transform import Rotation as R


# User options (change me)
# --------------- Setup options ---------------
tcp_host_ip = os.environ['UR5_IP'] # IP and port to robot arm as TCP client (UR5)
tcp_port = 30002
rtc_host_ip = os.environ['UR5_IP'] # IP and port to robot arm as real-time client (UR5)
rtc_port = 30003

# reduce z because none of them are available
# workspace_limits = np.asarray([[0.4, 0.648], [-.2, 0.3], [-.08, .05]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)

# do small amount of grid points to test the optimization code
workspace_limits = np.asarray([[0.4, 0.5], [0, 0.1], [-.08, 0]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)

calib_grid_step = 0.05 * 1
# checkerboard_offset_from_tool = [0,-0.13,0.02]

# position of the tool in the checkerboard frame
# checkerboard_offset_from_tool = [0,-0.13,0.02]

# -.015 is the z offset due to slanted calib board
checkerboard_offset_from_tool = [-.04,0.06,0.17-.015]

# from tool0:
# -4cm, 6cm, 17cm

# tool_orientation = [-np.pi/2,0,0] # [0,-2.22,2.22] # [2.22,2.22,0]
# tool_orientation = [0,0,0] # [0,-2.22,2.22] # [2.22,2.22,0]
# ---------------------------------------------


# Construct 3D calibration grid across workspace
gridspace_x = np.linspace(workspace_limits[0][0], workspace_limits[0][1], int(1 + (workspace_limits[0][1] - workspace_limits[0][0])//calib_grid_step))
gridspace_y = np.linspace(workspace_limits[1][0], workspace_limits[1][1], int(1 + (workspace_limits[1][1] - workspace_limits[1][0])//calib_grid_step))
gridspace_z = np.linspace(workspace_limits[2][0], workspace_limits[2][1], int(1 + (workspace_limits[2][1] - workspace_limits[2][0])//calib_grid_step))
calib_grid_x, calib_grid_y, calib_grid_z = np.meshgrid(gridspace_x, gridspace_y, gridspace_z)
num_calib_grid_pts = calib_grid_x.shape[0]*calib_grid_x.shape[1]*calib_grid_x.shape[2]
calib_grid_x.shape = (num_calib_grid_pts,1)
calib_grid_y.shape = (num_calib_grid_pts,1)
calib_grid_z.shape = (num_calib_grid_pts,1)
calib_grid_pts = np.concatenate((calib_grid_x, calib_grid_y, calib_grid_z), axis=1)

# measured_pts = []
# observed_pts = []
# observed_pix = []

p_WorldCharucocorner_Measured_dic = dict()
p_CameraCharucocorner_Estimated_dic = dict()
# observed_pix_dic = dict()

# Move robot to home pose
print('Connecting to robot...')
robot = Robot(False, None, None, workspace_limits,
              tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
              False, None, None, num_cameras=3)
# robot.open_gripper()

# Slow down robot
robot.joint_acc = 1.4
robot.joint_vel = 1.05

# Make robot gripper point upwards
robot.move_joints([-np.pi, -np.pi/2, np.pi/2, 0, np.pi/2, 0])


R_WorldTCPFrame_asrotvec = robot.get_tcp_pose(print_euler=True)[3:6]
# Move robot to each calibration point in workspace
print('Collecting data...')

# fig = plt.figure()

# check the workspace
check_workspace = False
# max x, midpoint y, midpoint z
if check_workspace:
    robot.move_to(np.array([workspace_limits[0][1],
                            (workspace_limits[1][1]+workspace_limits[1][0])/2,
                            (workspace_limits[2][0] + workspace_limits[2][1])/2]), R_WorldTCPFrame_asrotvec)
    time.sleep(3)

    # midpoint x, min y, midpoint z
    robot.move_to(np.array([(workspace_limits[0][0]+workspace_limits[0][1])/2,
                            workspace_limits[1][0],
                            (workspace_limits[2][0] + workspace_limits[2][1])/2]), R_WorldTCPFrame_asrotvec)
    time.sleep(3)

    # midpoint x, max y, midpoint z
    robot.move_to(np.array([(workspace_limits[0][0]+workspace_limits[0][1])/2,
                            workspace_limits[1][1],
                            (workspace_limits[2][0] + workspace_limits[2][1])/2]), R_WorldTCPFrame_asrotvec)
    time.sleep(3)

for calib_pt_idx in range(num_calib_grid_pts):
    t_WorldTCPFrame = calib_grid_pts[calib_pt_idx, :]

    # EFFECTIVELY, this is doing things in the [90 deg, 0, 90 deg] fixed world frame xyz rotation
    robot.move_to(t_WorldTCPFrame, R_WorldTCPFrame_asrotvec)
    time.sleep(.3)
    cam_data = robot.get_cameras_datas()

    time.sleep(.3)

    # Find charuco corner
    # color_img, depth_img = robot.camera.get_data()
    for serial_no, intrinsics, color_img, depth_img in cam_data:
        tf = charuco_util.get_charuco_tf(color_img, 0, intrinsics, np.zeros(4))

        # plt.subplot(211)
        # plt.imshow(color_img)
        # plt.subplot(212)
        #
        # plt.imshow(depth_img,cmap='nipy_spectral_r')
        # plt.show()

        if serial_no not in p_CameraCharucocorner_Estimated_dic.keys():
            assert serial_no not in p_WorldCharucocorner_Measured_dic.keys()
            # assert serial_no not in observed_pix_dic.keys()

            p_CameraCharucocorner_Estimated_dic[serial_no] = []
            p_WorldCharucocorner_Measured_dic[serial_no] = []
            # observed_pix_dic[serial_no] = []

        if tf is not None:
            print(f"Found tf at {t_WorldTCPFrame}")

            # check if the shapes are right
            # confirm this by drawing XY simple point and rotating
            # think about placing the translation in the world frame first. How do we get it to now align with the full
            # position in the world frame? We rotate it
            p_WorldCharucocorner_Measured_sample = R.from_rotvec(R_WorldTCPFrame_asrotvec).apply(t_WorldTCPFrame + checkerboard_offset_from_tool)

            # tf trans represents the charuco tag corner point in camera coordinates
            p_CameraCharucocorner_Estimated_dic[serial_no].append(tf[:3, :3] @ tf[:3, 3])

            p_WorldCharucocorner_Measured_dic[serial_no].append(p_WorldCharucocorner_Measured_sample)
            # observed_pix_dic[serial_no].append(color_img)

# Move robot back to home pose
robot.go_home()

for k in p_CameraCharucocorner_Estimated_dic.keys():
    p_CameraCharucocorner_Estimated_dic[k] = np.asarray(p_CameraCharucocorner_Estimated_dic[k])
    p_WorldCharucocorner_Measured_dic[k] = np.asarray(p_WorldCharucocorner_Measured_dic[k])
    # observed_pix_dic[k] = np.asarray(observed_pix_dic[k])


# Estimate rigid transform with SVD (from Nghia Ho)
def get_rigid_transform(A, B):
    """
    Gets the rigid transform from A to B

    which is also

    X_BA

    because we right multiply points in A to get points in B

    Args:
        A:
        B:

    Returns:

    """
    assert len(A) == len(B)
    N = A.shape[0] # Total points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - np.tile(centroid_A, (N, 1)) # Centre the points
    BB = B - np.tile(centroid_B, (N, 1))
    H = np.dot(np.transpose(AA), BB) # Dot is matrix multiplication for array
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0: # Special reflection case
       Vt[2,:] *= -1
       R = np.dot(Vt.T, U.T)
    t = np.dot(-R, centroid_A.T) + centroid_B.T
    return R, t


def get_rigid_transform_error(z_scale):
    global p_WorldCharucocorner_Measured, p_CameraCharucocorner_Estimated, X_CameraWorld, cam_intrinsics
    """
    measured_pts: num_samples, 3
    
    observed_pts: points in camera frame
    
    cam_intrinsics: 3, 3
    
    world2camera: 4, 4
    
    The reason why we have z scale is because the typical least squares Procrustes problem solution does not account 
    for scale.
    
    """

    # Apply z **scale** and compute new 3D observed points using camera intrinsics
    # observed_pix contains uv values for the corner points
    # observed_z = observed_pts[:,2:] * z_scale
    #
    # observed_x = np.multiply(observed_pix[:,[0]]-cam_intrinsics[0][2],observed_z/cam_intrinsics[0][0])
    #
    # observed_y = np.multiply(observed_pix[:,[1]]-cam_intrinsics[1][2],observed_z/cam_intrinsics[1][1])
    # new_observed_pts = np.concatenate((observed_x, observed_y, observed_z), axis=1)

    # Estimate rigid transform between measured points and new observed points
    # R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))

    import pdb
    pdb.set_trace()
    R_CameraWorld_Estimated, t_CameraWorld_Estimated = get_rigid_transform(np.asarray(p_WorldCharucocorner_Measured),
                                                       np.asarray(p_CameraCharucocorner_Estimated))

    t_CameraWorld_Estimated.shape = (3,1)

    # transformation from world to camera
    # OR pose of camera in world frame
    X_CameraWorld = np.concatenate((np.concatenate((R_CameraWorld_Estimated, t_CameraWorld_Estimated), axis=1), np.array([[0, 0, 0, 1]])), axis=0)

    # Compute rigid transform error by transforming the tool points into the camera frame
    p_CameraCharucocorner_ReconstructedFromMeasured = np.dot(R_CameraWorld_Estimated, np.transpose(p_WorldCharucocorner_Measured)) + \
                     np.tile(t_CameraWorld_Estimated, (1, p_WorldCharucocorner_Measured.shape[0]))

    error = np.transpose(p_CameraCharucocorner_ReconstructedFromMeasured) - np.asarray(p_CameraCharucocorner_Estimated)
    error = np.sum(np.multiply(error,error))
    rmse = np.sqrt(error / p_WorldCharucocorner_Measured.shape[0])
    return rmse


# optimize for each camera
for serial_no in p_CameraCharucocorner_Estimated_dic.keys():
    X_CameraWorld = np.eye(4)
    p_CameraCharucocorner_Estimated = p_CameraCharucocorner_Estimated_dic[serial_no]
    p_WorldCharucocorner_Measured = p_WorldCharucocorner_Measured_dic[serial_no]
    # observed_pix = observed_pix_dic[serial_no]
    cam_intrinsics = robot.serialno2intrinsics[serial_no]

    # Optimize z scale w.r.t. rigid transform error
    print('Calibrating...')
    z_scale_init = 1
    optim_result = optimize.minimize(get_rigid_transform_error, np.asarray(z_scale_init), method='Nelder-Mead')
    camera_depth_offset = optim_result.x

    # Save camera optimized offset and camera pose
    print('Saving...')
    np.savetxt(f"real/{serial_no}_camera_depth_scale.txt", camera_depth_offset, delimiter=' ')
    get_rigid_transform_error(camera_depth_offset)
    camera_pose = np.linalg.inv(X_CameraWorld)
    np.savetxt(f"real/{serial_no}_camera_pose.txt", camera_pose, delimiter=' ')
    print('Done.')