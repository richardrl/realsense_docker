#!/usr/bin/env python

import numpy as np
import time
from robot import Robot
from scipy import optimize
import os
import charuco_util
from scipy.spatial.transform import Rotation as R

"""
Start Config
"""
# User options (change me)
# --------------- Setup options ---------------
from utils.calibration_util import get_rigid_transform

tcp_host_ip = os.environ['UR5_IP'] # IP and port to robot arm as TCP client (UR5)
tcp_port = 30002
rtc_host_ip = os.environ['UR5_IP'] # IP and port to robot arm as real-time client (UR5)
rtc_port = 30003
SLEEP_TIME = 1
num_cameras = 4
traj_style = "ellipse"

# full calibration workspace
workspace_limits = np.asarray([[0.4, 0.648], [-.2, 0.3], [-.08, .05]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)

# do small amount of grid points to test the optimization code
# workspace_limits = np.asarray([[0.4, 0.5], [0, 0.1], [-.08, 0]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)

calib_grid_step = 0.05 * 1
# checkerboard_offset_from_tool = [0,-0.13,0.02]

# position of the tool in the checkerboard frame
# checkerboard_offset_from_tool = [0,-0.13,0.02]

# -.015 is the z offset due to slanted calib board
p_TCPFrameCharucocorner = np.array([-.04, 0.06, 0.17 - .015])

# check the workspace
check_workspace = False

num_ellipse_horizontal_samples = 24
# ellipsoid_a = .1
# ellipsoid_b = .2
# from tool0:
# -4cm, 6cm, 17cm

# tool_orien ation = [-np.pi/2,0,0] # [0,-2.22,2.22] # [2.22,2.22,0]
# tool_orientation = [0,0,0] # [0,-2.22,2.22] # [2.22,2.22,0]
# ---------------------------------------------
"""
End Config
"""

p_WorldCharucocorner_Measured_dic = dict()
p_CameraCharucocorner_Estimated_dic = dict()
# observed_pix_dic = dict()

# Move robot to home pose
print('Connecting to robot...')
robot = Robot(False, None, None, workspace_limits,
              tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
              False, None, None, num_cameras=num_cameras)
# robot.open_gripper()

# Slow down robot
robot.joint_acc = 1.4
robot.joint_vel = 1.05


# Move robot to each calibration point in workspace
print('Collecting data...')

# fig = plt.figure()


if traj_style == "ellipse":
    robot.go_home()
    X_ = robot.get_tcp_pose(print_euler=True)

    t_WorldTCPFrame_home, R_WorldTCPFrame_asrotvec_home = X_[0:3], X_[3:6]

    R_WorldTCPFrame_asrotvec_ellipsoid_home = (R.from_rotvec(R_WorldTCPFrame_asrotvec_home) * R.from_euler("z", -np.pi)).as_rotvec()
    t_WorldTCPFrame_ellipsoid_home = t_WorldTCPFrame_home + R.from_rotvec(R_WorldTCPFrame_asrotvec_home).apply(np.array([0, -.1, 0]))

    def get_ellipsoid_xy(t, a, b):
        return a*np.cos(t), b*np.sin(t)


    calib_grid_pts = []

    # rots as
    calib_grid_rotvecs = []
    # swivel up and down z in outer loop
    gridspace_z = np.linspace(workspace_limits[2][0]-.09, workspace_limits[2][1]-.09,
                              int(1 + (workspace_limits[2][1]-.09 - (workspace_limits[2][0]-.09))//calib_grid_step))

    # in loop, trace out discretized ellipsoid with 12 segments
    # the ellipsoid points are defined relative to the home position
    # also turn around the z-axis 360 degrees
    for z in gridspace_z:
        for ellipsoid_a in [.1, .125, .150]:
            for ellipsoid_b in [.2, .225, .25]:
                for t in np.linspace(0, 2*np.pi - .1, num_ellipse_horizontal_samples):
                    ellipsoid_pt_in_TCP_Frame = np.zeros(3)
                    ellipsoid_pt_in_TCP_Frame[:2] = get_ellipsoid_xy(t, ellipsoid_a, ellipsoid_b)
                    ellipsoid_pt_in_TCP_Frame[2] = z
                    calib_grid_pts.append(ellipsoid_pt_in_TCP_Frame + t_WorldTCPFrame_ellipsoid_home)

                    if t > np.pi - .1:
                        calib_grid_rotvecs.append((R.from_rotvec(R_WorldTCPFrame_asrotvec_ellipsoid_home) * R.from_euler("z", -(t - 2 * np.pi))).as_rotvec())
                    else:
                        calib_grid_rotvecs.append(
                            (R.from_rotvec(R_WorldTCPFrame_asrotvec_ellipsoid_home) * R.from_euler("z", -t)).as_rotvec())
    num_calib_grid_pts = len(calib_grid_pts)
    calib_grid_pts = np.array(calib_grid_pts)
    calib_grid_rotvecs = np.array(calib_grid_rotvecs)
elif traj_style == "grid":
    # Make robot gripper point upwards
    robot.move_joints([-np.pi, -np.pi/2, np.pi/2, 0, np.pi/2, 0])
    R_WorldTCPFrame_asrotvec_home = robot.get_tcp_pose(print_euler=True)[3:6]

    # max x, midpoint y, midpoint z
    # only implemented for gridstyle rn
    if check_workspace:
        robot.move_to(np.array([workspace_limits[0][1],
                                (workspace_limits[1][1] + workspace_limits[1][0]) / 2,
                                (workspace_limits[2][0] + workspace_limits[2][1]) / 2]), R_WorldTCPFrame_asrotvec_home)
        time.sleep(3)

        # midpoint x, min y, midpoint z
        robot.move_to(np.array([(workspace_limits[0][0] + workspace_limits[0][1]) / 2,
                                workspace_limits[1][0],
                                (workspace_limits[2][0] + workspace_limits[2][1]) / 2]), R_WorldTCPFrame_asrotvec_home)
        time.sleep(3)

        # midpoint x, max y, midpoint z
        robot.move_to(np.array([(workspace_limits[0][0] + workspace_limits[0][1]) / 2,
                                workspace_limits[1][1],
                                (workspace_limits[2][0] + workspace_limits[2][1]) / 2]), R_WorldTCPFrame_asrotvec_home)
        time.sleep(3)

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

for calib_pt_idx in range(num_calib_grid_pts):# Make robot gripper point upwards
    #
    # incrementally save data for debugging
    for serial_no in p_CameraCharucocorner_Estimated_dic.keys():
        np.savetxt(f"out/{serial_no}_p_CameraCharucocorner_Estimated.txt",
                   p_CameraCharucocorner_Estimated_dic[serial_no], delimiter=' ')

        np.savetxt(f"out/{serial_no}_p_WorldCharucocorner_Measured.txt",
                   p_WorldCharucocorner_Measured_dic[serial_no], delimiter=' ')

    t_WorldTCPFrame = calib_grid_pts[calib_pt_idx, :]

    # EFFECTIVELY, this is doing things in the [90 deg, 0, 90 deg] fixed world frame xyz rotation
    if traj_style == 'grid':
        robot.move_to(t_WorldTCPFrame, R_WorldTCPFrame_asrotvec_home)
    else:
        # traj_style == ellipse
        robot.move_to(t_WorldTCPFrame, calib_grid_rotvecs[calib_pt_idx, :])

    time.sleep(SLEEP_TIME)
    cam_data = robot.get_cameras_datas()

    # time.sleep(SLEEP_TIME/2)

    if calib_pt_idx % num_ellipse_horizontal_samples == 0:
        # this logic is to avoid the joint limits
        robot.go_home()

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
            print(f"Found tf at {t_WorldTCPFrame} for {serial_no}")

            # check if the shapes are right
            # confirm this by drawing XY simple point and rotating
            # think about placing the translation in the world frame first. How do we get it to now align with the full
            # position in the world frame? We rotate it
            # TODO: the below code is wrong

            # tf trans represents the charuco tag corner point in camera coordinates
            p_CameraCharucocorner_Estimated_dic[serial_no].append(tf[:3, 3])

            X_WorldTCPFrame = np.zeros((4, 4))
            X_WorldTCPFrame[3, 3] = 1
            X_WorldTCPFrame[:3, :3] = R.from_rotvec(calib_grid_rotvecs[calib_pt_idx, :]).as_matrix()
            X_WorldTCPFrame[:3, 3] = t_WorldTCPFrame

            p_WorldCharucocorner_Measured_sample = (X_WorldTCPFrame @ np.concatenate([p_TCPFrameCharucocorner, [1]]))[:3]

            # p_WorldCharucocorner_Measured_sample = t_WorldTCPFrame + p_TCPFrameCharucocorner
            p_WorldCharucocorner_Measured_dic[serial_no].append(p_WorldCharucocorner_Measured_sample)
            # observed_pix_dic[serial_no].append(color_img)

# Move robot back to home pose
robot.go_home()

for k in p_CameraCharucocorner_Estimated_dic.keys():
    p_CameraCharucocorner_Estimated_dic[k] = np.asarray(p_CameraCharucocorner_Estimated_dic[k])
    p_WorldCharucocorner_Measured_dic[k] = np.asarray(p_WorldCharucocorner_Measured_dic[k])
    # observed_pix_dic[k] = np.asarray(observed_pix_dic[k])


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

    # for some reason, if we only have one point, it doesn't seem to work..
    assert p_CameraCharucocorner_Estimated.shape[0] > 1

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
    np.savetxt(f"out/{serial_no}_camera_depth_scale.txt", camera_depth_offset, delimiter=' ')
    get_rigid_transform_error(camera_depth_offset)
    X_WorldCamera = np.linalg.inv(X_CameraWorld)
    np.savetxt(f"out/{serial_no}_camera_pose.txt", X_WorldCamera, delimiter=' ')
    print('Done.')