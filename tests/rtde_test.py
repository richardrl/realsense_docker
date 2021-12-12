import rtde_control
import rtde_receive
import numpy as np

import robotiq_gripper
import os

gripper = robotiq_gripper.RobotiqGripper()

gripper.connect(os.environ['UR5_IP'], 63352)

gripper.activate()

def log_info(gripper):
    print(f"Pos: {str(gripper.get_current_position()): >3}  "
          f"Open: {gripper.is_open(): <2}  "
          f"Closed: {gripper.is_closed(): <2}  ")

gripper.move_and_wait_for_pos(255, 255, 255)
log_info(gripper)
gripper.move_and_wait_for_pos(0, 255, 255)
log_info(gripper)


rtde_c = rtde_control.RTDEControlInterface(os.environ['UR5_IP'])

rtde_r = rtde_receive.RTDEReceiveInterface(os.environ['UR5_IP'])

target = rtde_r.getActualTCPPose()
target[2] -= .1

home_joint_config = [-(180.0/360.0)*2*np.pi, -(84.2/360.0)*2*np.pi, (112.8/360.0)*2*np.pi, -(119.7/360.0)*2*np.pi, -(90.0/360.0)*2*np.pi, 0.0]

# rtde_c.moveL(target, 0.01, 0.01)
rtde_c.moveJ(home_joint_config, speed=.1, acceleration=.01)

# RobotiqGripper gripper("127.0.0.1", 63352, true);
#   gripper.connect();