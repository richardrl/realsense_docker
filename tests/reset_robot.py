from robot import Robot
import os
import numpy as np


workspace_limits = np.asarray([[0.4, 0.648], [-.2, 0.3], [-.08, .05]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)

tcp_host_ip = os.environ['UR5_IP'] # IP and port to robot arm as TCP client (UR5)
tcp_port = 30002
rtc_host_ip = os.environ['UR5_IP'] # IP and port to robot arm as real-time client (UR5)
rtc_port = 30003
SLEEP_TIME = 1
traj_style = "ellipse"
robot = Robot(False, None, None, workspace_limits,
              tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
              False, None, None, num_cameras=0)

import pdb
pdb.set_trace()