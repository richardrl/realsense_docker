class Robot(object):
    def __init__(self, is_sim, obj_mesh_dir, num_obj, workspace_limits,
                 tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
                 is_testing, test_preset_cases, test_preset_file):
        self.workspace_limits = workspace_limits
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

        self.close_gripper()
        self.go_home()

    def close_gripper(self):
