import numpy as np
import matplotlib.pyplot as plt
from utils.calibration_util import get_rigid_transform
import open3d
from utils import visualization_util


observed_pts = np.loadtxt('../out/949322060350_p_CameraCharucocorner_Estimated', delimiter=' ')
measured_pts = np.loadtxt('../out/949322060350_p_WorldCharucocorner_Measured', delimiter=' ')

# visualize the charuco points in camera frame
open3d.visualization.draw_geometries([visualization_util.make_point_cloud_o3d(observed_pts[observed_pts[:, 2] < 1],
                                         [1, 0, 0],
                                                                              normalize_color=True),
                                      open3d.geometry.TriangleMesh.create_coordinate_frame(.03, [0, 0, 0])])

# calculate X_CW
R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(observed_pts))
print("Debug live calculated R and t")
print(R)
print(t)

# visualize things in world frame
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(measured_pts[:,0], measured_pts[:,1], measured_pts[:,2], c='red')

X_CW = np.zeros((4, 4))
X_CW[:3, :3] = R.copy()
X_CW[:3, 3] = t.copy()
X_CW[3, 3] = 1

world_transformed_observed_pts = (np.linalg.inv(X_CW)[:3, :3] @ observed_pts.T).T + np.linalg.inv(X_CW)[:3, 3]
ax.scatter(world_transformed_observed_pts[:,0], world_transformed_observed_pts[:,1], world_transformed_observed_pts[:,2], c='blue')


# visualize things in camera frame
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(observed_pts[:,0], observed_pts[:,1], observed_pts[:,2], c='green')

# apply X_CW
new_observed_pts = (R @ measured_pts.T).T + t
ax.scatter(new_observed_pts[:,0], new_observed_pts[:,1], new_observed_pts[:,2], c='blue')

# apply X_CW from file
X_WC_from_file = np.loadtxt("../out/949322060350_camera_pose.txt", delimiter=' ')
X_CW_from_file = np.linalg.inv(X_WC_from_file)
new_observed_pts_2 = (X_CW_from_file[:3, :3] @ measured_pts.T).T + X_CW_from_file[:3, 3]
ax.scatter(new_observed_pts_2[:,0], new_observed_pts_2[:,1], new_observed_pts_2[:,2], c='red')

plt.show()

# print(camera_depth_offset)
# R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(observed_pts))
# t.shape = (3,1)
# camera_pose = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)
# camera2robot = np.linalg.inv(camera_pose)
# t_observed_pts = np.transpose(np.dot(camera2robot[0:3,0:3],np.transpose(observed_pts)) + np.tile(camera2robot[0:3,3:],(1,observed_pts.shape[0])))

# ax.scatter(t_observed_pts[:,0],t_observed_pts[:,1],t_observed_pts[:,2], c='red')

# new_observed_pts = observed_pts.copy()
# new_observed_pts[:,2] = new_observed_pts[:,2] * camera_depth_offset[0]
# R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))
# t.shape = (3,1)
# camera_pose = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)
# camera2robot = np.linalg.inv(camera_pose)
# t_new_observed_pts = np.transpose(np.dot(camera2robot[0:3,0:3],np.transpose(new_observed_pts)) + np.tile(camera2robot[0:3,3:],(1,new_observed_pts.shape[0])))
#
# ax.scatter(t_new_observed_pts[:,0],t_new_observed_pts[:,1],t_new_observed_pts[:,2], c='green')

