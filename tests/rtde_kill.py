import rtde_control
import rtde_receive
import numpy as np


rtde_c = rtde_control.RTDEControlInterface("128.30.16.198")

rtde_r = rtde_receive.RTDEReceiveInterface("128.30.16.198")

rtde_c.stopScript()