import cv2
import numpy as np
from cv2 import aruco


def get_charuco_tf(image, board_num, camera_matrix, dist_coeffs):
    board, dictionary = make_board(board_num)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(image, dictionary, parameters=parameters)
    aruco.refineDetectedMarkers(image, board, corners, ids, rejectedImgPoints)
    retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, None, None)
    if type(ids) != type(None):
        charucoretval, charucoCorners, charucoIds = aruco.interpolateCornersCharuco(corners, ids, image, board)
        retval, rvec, tvec = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, board, camera_matrix,
                                                            dist_coeffs, None, None)

    if retval:
        tf = make_tf(rvec, tvec)
    else:
        tf = None

    return tf


def make_tf(rvec, tvec):
    tf = np.eye(4)
    tf[:3, :3] = cv2.Rodrigues(rvec)[0]
    tf[:3, -1] = tvec.squeeze()
    return tf


def make_rodrigues(transform):
    rvec = cv2.Rodrigues(transform[:3, :3])[0]
    tvec = transform[:3, -1].reshape(3, 1)
    return rvec, tvec


def draw_axis(img, transform, mtx, dist_coeffs, marker_size=0.075):
    img = img.copy()
    rvec, tvec = make_rodrigues(transform)
    img = aruco.drawAxis(img, mtx, dist_coeffs, rvec, tvec, marker_size)
    return img