import rtde_control
import rtde_receive
import os
import numpy as np
import robotiq_gripper
from scipy.spatial.transform import Rotation as R


rtde_c = rtde_control.RTDEControlInterface(os.environ['UR5_IP'])

rtde_r = rtde_receive.RTDEReceiveInterface(os.environ['UR5_IP'])


class Robot(object):
    def __init__(self, is_sim, obj_mesh_dir, num_obj, workspace_limits,
                 tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
                 is_testing, test_preset_cases, test_preset_file, load_camera=False):
        self.workspace_limits = workspace_limits

        if load_camera:
            from real.camera import Camera
            self.camera = Camera()
            self.cam_intrinsics = self.camera.intrinsics

        # Load camera pose (from running calibrate.py), intrinsics and depth scale
        # self.cam_pose = np.loadtxt('real/camera_pose.txt', delimiter=' ')
        # self.cam_depth_scale = np.loadtxt('real/camera_depth_scale.txt', delimiter=' ')


        self.home_joint_config = [-(180.0 / 360.0) * 2 * np.pi, -(84.2 / 360.0) * 2 * np.pi,
                                  (112.8 / 360.0) * 2 * np.pi, -(119.7 / 360.0) * 2 * np.pi,
                                  -(90.0 / 360.0) * 2 * np.pi, 0.0]
        self.joint_acc = 1.4  # Safe: 1.4
        self.joint_vel = 1.05  # Safe: 1.05

        # Joint tolerance for blocking calls
        self.joint_tolerance = 0.01

        # Default tool speed configuration
        self.tool_acc = 1.5  # Safe: 0.5
        self.tool_vel = 0.2  # Safe: 0.2

        # Tool pose tolerance for blocking calls
        self.tool_pose_tolerance = [0.002, 0.002, 0.002, 0.01, 0.01, 0.01]

        self.gripper = robotiq_gripper.RobotiqGripper()

        self.gripper.connect(os.environ['UR5_IP'], 63352)

        self.gripper.activate()

        # self.close_gripper()
        self.go_home()

    def open_gripper(self):
        self.gripper.move_and_wait_for_pos(0, 255, 255)

    def close_gripper(self):
        self.gripper.move_and_wait_for_pos(255, 255, 255)

    def go_home(self):
        rtde_c.moveJ(self.home_joint_config, speed=self.joint_vel, acceleration=self.joint_acc)

    def get_camera_data(self):
        color_img, depth_img = self.camera.get_data()
        return color_img, depth_img

    def get_tcp_pose(self, print_euler=False):
        """

        Returns: (x,y,z,rx,ry,rz)

        """
        tcp_pose = rtde_r.getActualTCPPose()
        if print_euler:
            print(f"xyz euler: {R.from_rotvec(tcp_pose[3:6]).as_euler('xyz')}")
            print(f"zyx euler: {R.from_rotvec(tcp_pose[3:6]).as_euler('zyx')}")

        return tcp_pose

    def move_to(self, tool_position, tool_orientation):
        """
        Move the ee
        Args:
            tool_position:
            tool_orientation:

        Returns:

        """
        target = np.concatenate([tool_position, tool_orientation])
        # (x,y,z,rx,ry,rz)
        rtde_c.moveL(target, self.tool_vel, self.tool_acc)

    def move_joints(self, q_set):
        rtde_c.moveJ(q_set, speed=self.joint_vel, acceleration=self.joint_acc)