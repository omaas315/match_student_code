import pdb

import numpy as np

# all functions regarding tinyspline without dependency on tinyspline lib


def calc_connection_points(sparse_path):
    """Calculates connection points in the middle of given path; start and goal are not modified.

    Args:
        sparse_path (np.NDArray): sparse path of shape (2, n)

    Returns:
        np.NDArray: Connection points of shape (2, n-1)
    """
    prev_point = sparse_path.T[0]
    connection_points = []
    eps = 0.0001
    for i, point in enumerate(sparse_path.T):
        if abs(point[0] - prev_point[0]) < eps and abs(point[1] - prev_point[1]) < eps:
            continue
        if i == 1:
            # first point
            connection_point = prev_point.tolist()
        elif i == len(sparse_path.T) - 1:
            # last point
            connection_point = point.tolist()
        else:
            connection_point = [
                (prev_point[0] + point[0]) / 2,
                (prev_point[1] + point[1]) / 2,
            ]
        connection_points.append(connection_point)
        prev_point = point
    return np.asarray(connection_points, dtype=np.float64).T


def calc_control_points_for_path(sparse_path, lengths):
    """Calculate control points for every segment of the given path.

    Args:
        sparse_path (np.NDArray): sparse path of shape (2, n)
        lengths (np.NDArray): desired lengths for the tangents of each segment shape (n-1,)

    Returns:
        np.NDArray: spline control points of shape (n-1, 2)
    """
    connection_points = calc_connection_points(sparse_path)
    tangents = np.diff(sparse_path.T, axis=0)
    tangents /= np.sqrt(np.sum(tangents**2, axis=1)).reshape(-1, 1)
    tangents *= lengths.reshape(-1, 1)

    distances = np.sqrt(np.sum(np.diff(connection_points.T, axis=0) ** 2, axis=1))
    d_AB = distances[:-1]
    d_BC = distances[1:]
    alphas = (d_BC / (d_AB + d_BC)).reshape(-1, 1)
    betas = (d_AB / (d_AB + d_BC)).reshape(-1, 1)
    As = connection_points.T[:-2]
    Bs = connection_points.T[1:-1]
    Cs = connection_points.T[2:]
    tangents_A = tangents[:-2]
    tangents_B = tangents[1:-1]
    tangents_C = tangents[2:]
    intermediate_accelerations = alphas * (
        6 * As + 2 * tangents_A + 4 * tangents_B - 6 * Bs
    ) + betas * (-6 * Bs - 4 * tangents_B - 2 * tangents_C + 6 * Cs)
    first_acc = (
        (-6 * Bs[0] - 4 * tangents_B[0] - 2 * tangents_C[0] + 6 * Cs[0])
        .reshape(-1, 1)
        .T
    )
    last_acc = (
        (6 * As[-1] + 2 * tangents_A[-1] + 4 * tangents_B[-1] - 6 * Bs[-1])
        .reshape(-1, 1)
        .T
    )
    accelerations = np.concatenate((first_acc, intermediate_accelerations, last_acc))
    p_0s = connection_points.T[:-1]
    p_5s = connection_points.T[1:]
    p_1s = 0.2 * tangents[:-1] + p_0s  # * 5.0 * 0.5 * ratio
    p_4s = p_5s - 0.2 * tangents[1:]  # * 5.0 * 0.5 * ratio
    p_2s = 0.05 * accelerations[:-1] + 2 * p_1s - p_0s
    p_3s = 0.05 * accelerations[1:] + 2 * p_4s - p_5s
    control_points = np.stack((p_0s, p_1s, p_2s, p_3s, p_4s, p_5s), axis=1)
    return control_points


def calc_control_points_for_path_orig(sparse_path, lengths):
    """Calculate control points for every segment of the given path. Uses sparse path directly.

    Args:
        sparse_path (np.NDArray): sparse path of shape (2, n)
        lengths (np.NDArray): desired lengths for the tangents of each segment shape (n-1,)

    Returns:
        np.NDArray: spline control points of shape (n-1, 2)
    """
    first_tangent = (sparse_path.T[1] - sparse_path.T[0]) * 1.0
    tangents = [first_tangent]
    for idx, point in enumerate(sparse_path.T[1:-1]):
        ab = point - sparse_path.T[idx]
        ac = sparse_path.T[idx + 2] - point
        ab_norm = ab / np.linalg.norm(ab)
        ac_norm = ac / np.linalg.norm(ac)
        anglular_bisector = ab_norm + ac_norm
        anglular_bisector /= np.linalg.norm(anglular_bisector)
        length_diff = np.linalg.norm(ac)
        tangent = length_diff * anglular_bisector * 0.5
        tangents.append(tangent)
    last_tangent = (sparse_path.T[-1] - sparse_path.T[-2]) * 0.5
    tangents.append(last_tangent)
    tangents = np.asarray(tangents)
    tangents *= lengths.reshape(-1, 1)

    distances = np.sqrt(np.sum(np.diff(sparse_path.T, axis=0) ** 2, axis=1))
    d_AB = distances[:-1]
    d_BC = distances[1:]
    alphas = (d_BC / (d_AB + d_BC)).reshape(-1, 1)
    betas = (d_AB / (d_AB + d_BC)).reshape(-1, 1)
    As = sparse_path.T[:-2]
    Bs = sparse_path.T[1:-1]
    Cs = sparse_path.T[2:]
    tangents_A = tangents[:-2]
    tangents_B = tangents[1:-1]
    tangents_C = tangents[2:]
    intermediate_accelerations = alphas * (
        6 * As + 2 * tangents_A + 4 * tangents_B - 6 * Bs
    ) + betas * (-6 * Bs - 4 * tangents_B - 2 * tangents_C + 6 * Cs)
    first_acc = (
        (-6 * Bs[0] - 4 * tangents_B[0] - 2 * tangents_C[0] + 6 * Cs[0])
        .reshape(-1, 1)
        .T
    )
    last_acc = (
        (6 * As[-1] + 2 * tangents_A[-1] + 4 * tangents_B[-1] - 6 * Bs[-1])
        .reshape(-1, 1)
        .T
    )
    accelerations = np.concatenate((first_acc, intermediate_accelerations, last_acc))
    p_0s = sparse_path.T[:-1]
    p_5s = sparse_path.T[1:]
    p_1s = 0.2 * tangents[:-1] + p_0s  # * 5.0 * 0.5 * ratio
    p_4s = p_5s - 0.2 * tangents[1:]  # * 5.0 * 0.5 * ratio
    p_2s = 0.05 * accelerations[:-1] + 2 * p_1s - p_0s
    p_3s = 0.05 * accelerations[1:] + 2 * p_4s - p_5s
    control_points = np.stack((p_0s, p_1s, p_2s, p_3s, p_4s, p_5s), axis=1)
    return control_points


def get_coefficient_matrices(control_points):
    C_QBS = []
    for cps in control_points:
        C_QB = np.array(
            [
                -cps[0] + 5 * cps[1] - 10 * cps[2] + 10 * cps[3] - 5 * cps[4] + cps[5],
                5 * cps[0] - 20 * cps[1] + 30 * cps[2] - 20 * cps[3] + 5 * cps[4],
                -10 * cps[0] + 30 * cps[1] - 30 * cps[2] + 10 * cps[3],
                10 * cps[0] - 20 * cps[1] + 10 * cps[2],
                -5 * cps[0] + 5 * cps[1],
                cps[0],
            ]
        )
        C_QBS.append(C_QB)
    return np.asarray(C_QBS)


def get_sample(C_QB, frac):
    T = np.array([frac**5, frac**4, frac**3, frac**2, frac, 1])
    T_der = np.array([5 * frac**4, 4 * frac**3, 3 * frac**2, 2 * frac, 1, 0])
    T_der2 = np.array([20 * frac**3, 12 * frac**2, 6 * frac, 2, 0, 0])
    sample = T @ C_QB
    sample_der = T_der @ C_QB
    sample_der2 = T_der2 @ C_QB
    curvature = (sample_der[0] * sample_der2[1] - sample_der2[0] * sample_der[1]) / (
        (sample_der[0] ** 2 + sample_der[1] ** 2) ** 1.5
    )
    return sample, sample_der, sample_der2, curvature


def sample_spline_path(control_points, num_samples=1000):
    """Sample spline path from its control points

    Args:
        control_points (np.NDArray): output of @calc_control_points_for_path
        num_samples (int, optional): Number of samples per segment. Defaults to 1000.

    Returns:
        np.NDArray: output samples of shape (2, n)
    """
    samples = []
    samples_der = []
    samples_der2 = []
    curvatures = []
    fracs = np.linspace(0.0, 1.0, num_samples)

    C_QBS = get_coefficient_matrices(control_points)
    for C_QB in C_QBS:
        for frac in fracs[:-1]:
            sample, sample_der, sample_der2, curvature = get_sample(C_QB, frac)
            samples.append(sample)
            samples_der.append(sample_der)
            samples_der2.append(sample_der2)
            curvatures.append(curvature)
    sample, sample_der, sample_der2, curvature = get_sample(C_QBS[-1], fracs[-1])
    samples.append(sample)
    samples_der.append(sample_der)
    samples_der2.append(sample_der2)
    curvatures.append(curvature)
    samples = np.asarray(samples).T
    samples_der = np.asarray(samples_der).T
    samples_der2 = np.asarray(samples_der2).T
    return samples, samples_der, samples_der2, curvatures


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
