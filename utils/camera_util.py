import numpy as np

# https://dev.intelrealsense.com/docs/projection-in-intel-realsense-sdk-20


def convert_depth_to_pointcloud(depth_ims, camera_ext, camera_int):
    """
    This only works if there was no image segmentation done
    :param depth_ims: m x n
    :param camera_ext: 4 x 4
    :param camera_int: 3 x 3
    :return:
        pointcloud: (mxn) x 3
    """

    # default setting is 720 x 1280
    # 1280 is the width
    # 720 is the height

    # according to docs
    # x axis points to the right
    # the y axis points down
    img_height, img_width = depth_ims.shape

    # -> 2 x 720 x 1280
    uv_coords = np.mgrid[0: img_height,
               0: img_width].reshape(2, -1)

    # this switches the order
    uv_coords[[0, 1], :] = uv_coords[[1, 0], :]

    # 2x(width*height) -> 3x(width*height)
    p_uv_one_C2D = np.concatenate((uv_coords,
                              np.ones((1, uv_coords.shape[1]))))

    # inverse of intrinsics matrix is
    # 1/f   | 1 0 -ppx  |
    #       | 0 1 -ppy  |
    #       | 0 0 f     |

    p_uv_one_C3D_dividedbyZ = np.dot(np.linalg.inv(camera_int), p_uv_one_C2D)


    # remember: depth_ims.flatten is basically the corresponding Z
    p_scene_C = np.multiply(p_uv_one_C3D_dividedbyZ, depth_ims.flatten())

    p_scene_one_C = np.concatenate((p_scene_C,
                    np.ones((1, p_scene_C.shape[1]))),
                   axis=0)

    # (4, num_points) = (4, 4) dot (4, num_points)
    # camera_ext is X_WC
    p_scene_one_W = np.dot(camera_ext, p_scene_one_C)

    return p_scene_one_W[:3, :].T, p_scene_C.T