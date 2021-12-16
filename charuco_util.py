import cv2
import numpy as np
from cv2 import aruco


def make_board(board_num, num_rows=4, num_cols=4, square_size=0.024, marker_size=0.018):
    num_markers = int(np.ceil(num_rows * num_cols / 2))
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
    dictionary.bytesList = dictionary.bytesList[board_num*num_markers:board_num*num_markers+num_markers,...]
    board = aruco.CharucoBoard_create(num_cols,num_rows,square_size,marker_size,dictionary)

    # cv2.imshow('image', board.draw((370, 370)))
    # cv2.waitKey(0)

    return board, dictionary


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
        tf = make_tf(rvec, tvec).copy()
    else:
        # print("Board not detected!")
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

    # OpenCV does the axis as bgr instead of rgb
    img = aruco.drawAxis(img, mtx, dist_coeffs, rvec, tvec, marker_size)
    return img