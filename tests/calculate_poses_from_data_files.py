# using this temporarily because calibrate is making the wrong poses
# run this from tests directory, or the savetxt wont work

import glob
from pathlib import Path
import numpy as np
from utils.calibration_util import get_rigid_transform


txts = [f for f in glob.glob(str(Path("../out/") / "*p_CameraCharucocorner_Estimated*"), recursive=True)]

assert txts

for txt in txts:
    if "p_CameraCharucocorner_Estimated" in txt:
        serial_no = txt.split("_")[0].split("/")[-1]
        print(f"Calculating for {serial_no}")
        observed_pts = np.loadtxt(f'../out/{serial_no}_p_CameraCharucocorner_Estimated', delimiter=' ')
        measured_pts = np.loadtxt(f'../out/{serial_no}_p_WorldCharucocorner_Measured', delimiter=' ')

        R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(observed_pts))

        X_CameraWorld = np.zeros((4, 4))
        X_CameraWorld[3, 3] = 1
        X_CameraWorld[:3, :3] = R.copy()
        X_CameraWorld[:3, 3] = t.copy()

        X_WorldCamera = np.linalg.inv(X_CameraWorld)
        print("X_WorldCamera")
        print(X_WorldCamera)
        np.savetxt(f"../out/{serial_no}_camera_pose.txt", X_WorldCamera, delimiter=' ')

        print("X_WorldCamera_loaded")
        X_WorldCamera_loaded = np.loadtxt(f"../out/{serial_no}_camera_pose.txt", delimiter=' ')
        print(X_WorldCamera_loaded)