import numpy as np


def points_from_path(inp_path):
    """type conversion from PoseStampd to numpy array

    Args:
        inp_path (list of PoseStamped): input path as list of PoseStamped

    Returns:
        np.NDArray: output path as array
    """
    out_path = []
    # pdb.set_trace()
    for pose in inp_path.poses:
        point = [pose.pose.position.x, pose.pose.position.y]
        out_path.append(point)
    return np.asarray(out_path, dtype=np.float64).T


def calc_derivative(samples):
    """Calculate numerical derivative of a timeseries.

    Args:
        samples (np.NDArary): input samples of shape (2, n)

    Returns:
        np.NDArray: derivation for each value (2, n-1)
    """
    der = np.diff(samples)
    der_lengths = np.sqrt(np.sum(der**2, axis=0))
    der /= der_lengths
    return der


def calc_angles(points):
    """Calculate Angles for each point in path as angle to next point.

    Args:
        points (np.NDArray): path of shape (2, n)

    Returns:
        _type_: angles of shape (n,)
    """
    deltas = np.diff(points)
    angles = np.arctan2(deltas[1], deltas[0])
    new_angles = []
    for i in range(len(angles)):
        if i == 0:
            new_angles.append(angles[i])
            continue
        delta_angle = angles[i] - new_angles[-1]
        angle_diff = np.arctan2(np.sin(delta_angle), np.cos(delta_angle))
        # if delta_angle < 0:
        #     delta_angle = -(-delta_angle % np.pi)
        # else:
        #     delta_angle = delta_angle % np.pi
        new_angles.append(new_angles[-1] + angle_diff)
    return np.array(new_angles)


def offset_path(path, angles, offset):
    """transforms each point of input path by the offset

    Args:
        path (np.NDArray): input path of shape (2, n)
        angles (np.NDArray): angles for each point (n,)
        offset (Tuple[float, float]): desired relative offset to path (x, y)

    Returns:
        np.NDArray: path with offset
    """
    res_path_x = []
    res_path_y = []
    for point, angle in zip(path.T, angles):
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        tf_point_x = cos_angle * offset[0] - sin_angle * offset[1] + point[0]
        tf_point_y = sin_angle * offset[0] + cos_angle * offset[1] + point[1]
        res_path_x.append(tf_point_x)
        res_path_y.append(tf_point_y)
    res_path = np.asarray([res_path_x, res_path_y], dtype=np.float64)
    return res_path


def world_to_map(point_world, map_origin, map_size_x, map_size_y, map_resolution):
    """transforms point from world frame to map frame.

    Args:
        point_world: 2d point in m
        map_origin: 2d map origin
        map_size_x: x dimension of map in px
        map_size_y: y dimension of map in px
        map_resolution: resolution of map in m/px

    Returns:
        2d point in map frame (px)
    """
    point_map = [0, 0]
    if point_world[0] < map_origin[0] or point_world[1] < map_origin[1]:
        return point_map

    point_map[0] = int((point_world[0] - map_origin[0]) / map_resolution)
    point_map[1] = int((point_world[1] - map_origin[1]) / map_resolution)

    if point_map[0] < map_size_x and point_map[1] < map_size_y:
        return point_map

    return [map_size_x - 1, map_size_y - 1]
