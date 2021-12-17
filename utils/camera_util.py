import numpy as np

# https://dev.intelrealsense.com/docs/projection-in-intel-realsense-sdk-20


def convert_depth_to_pointcloud(depth_ims, camera_ext, camera_int):
    """
    This only works if there was no image segmentation done
    :param depth_ims: m x n
    :param camera_ext: 4 x 4. X_WC
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

    # -> 2 x 720 x 1280 -> 2 X (720 * 1280)
    # the reshape tiled the 1280d row vector repeatedly in the top

    uv_coords = np.mgrid[0: img_height,
               0: img_width].reshape(2, -1)

    # uv_coords[0] is the outer loop... the first 1280 elements are 0, the second 1280 elements are 1, etc...
    # uv_coords[1] is tiled 1280

    # this switches the order
    # by doing this swap, we correctly make the 1280 the max x, and the 720 is the max y
    # the first 1280 elements are looping over the img_width==1280 because before the switch the first 720 elemets
    # were looping over the img_height=720


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
    # depth_ims.flatten() is equivalent to putting the rows (the width) first
    # therefore, depth_ims.flatten() iterates in the in the outer loop over row idxs,
    # in the inner loop over col idxs
    # therefore the inner loop is over img_width==1280
    # p_uv_one_C3D_dividedbyZ the first 1280 elements are also looping over img_width
    # therefore we have a match, because looping over the row idxs, is the same as looping over the img_width
    p_scene_C = np.multiply(p_uv_one_C3D_dividedbyZ, depth_ims.flatten())

    p_scene_one_C = np.concatenate((p_scene_C,
                    np.ones((1, p_scene_C.shape[1]))),
                   axis=0)

    # (4, num_points) = (4, 4) dot (4, num_points)
    # camera_ext is X_WC
    p_scene_one_W = np.dot(camera_ext, p_scene_one_C)

    return p_scene_one_W[:3, :].T, p_scene_C.T