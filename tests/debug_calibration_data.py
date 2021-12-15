import numpy as np
import matplotlib.pyplot as plt
from utils.calibration_util import get_rigid_transform


observed_pts = np.loadtxt('../out/949322060350_p_CameraCharucocorner_Estimated', delimiter=' ')
measured_pts = np.loadtxt('../out/949322060350_p_WorldCharucocorner_Measured', delimiter=' ')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(measured_pts[:,0],measured_pts[:,1],measured_pts[:,2], c='red')


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(observed_pts[:,0],observed_pts[:,1],observed_pts[:,2], c='green')

# calculate X_CW
R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(observed_pts))

# apply X_CW


new_observed_pts = (R @ measured_pts.T).T + t
ax.scatter(new_observed_pts[:,0],new_observed_pts[:,1],new_observed_pts[:,2], c='blue')

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

