import numpy as np

def get_rotMat(q):
    rotMat = np.array([
        [1 - 2*q[2]**2 - 2*q[3]**2, 2*(q[1]*q[2] - q[0]*q[3]), 2*(q[1]*q[3] + q[0]*q[2])],
        [2*(q[1]*q[2] + q[0]*q[3]), 1 - 2*q[1]**2 - 2*q[3]**2, 2*(q[2]*q[3] - q[0]*q[1])],
        [2*(q[1]*q[3] - q[0]*q[2]), 2*(q[2]*q[3] + q[0]*q[1]), 1 - 2*q[1]**2 - 2*q[2]**2]
    ])
    return rotMat

def quat_mult(q1, q2):
    return np.array([
        q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],
        q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],
        q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1],
        q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
    ])

def quat_derivative(q, w):
    return 0.5 * quat_mult(np.array([0, *w]), q)

def MRP2quat(sigma, ref_quat=None):
    """
    Converts Modified Rodrigues Parameters (MRP) vector sigma
    to quaternion vector q.

    Parameters
    ----------
    sigma : array_like, shape (3,)
        MRP vector.

    Returns
    -------
    q : ndarray, shape (4,)
        Quaternion [qw, qx, qy, qz].
    """
    sigma = np.asarray(sigma)
    s2 = np.dot(sigma, sigma)
    # if s2 > 1:
    #     sigma = -sigma / s2
    #     s2 = np.dot(sigma, sigma)
    denom = 1 + s2
    qw = (1 - s2) / denom
    qx = 2 * sigma[0] / denom
    qy = 2 * sigma[1] / denom
    qz = 2 * sigma[2] / denom
    quat = np.hstack((qw, qx, qy, qz))
    quat = quat / np.linalg.norm(quat)

    if ref_quat is not None and np.dot(quat, ref_quat) < 0:
        # Enforce sign consistency
        quat = -quat
    return quat

def sample_other_path(
    t0: float,
    dt: float,
    Nx: int,
    t_other,
    pos_other,
    vel_other=[np.zeros(3)],
    acc_other=[np.zeros(3)]
) -> np.ndarray:
    """
    Samples other agent's predicted path over time horizon using interpolation and
    constant acceleration extrapolation.

    Args:
        t0 (float): Current time [ns]
        dt (float): Time step [s]
        Nx (int): Prediction horizon length
        t_other (float or list[float]): Timestamps [ns]
        pos_other (np.array or list[np.array]): Positions
        vel_other (list[np.array]): Optional velocities (default: zero)
        acc_other (list[np.array]): Optional accelerations (default: zero)

    Returns:
        np.ndarray: (Nx, 3) array of predicted positions
    """
    # Ensure input is in list form
    if not isinstance(t_other, list):
        t_other = [t_other]
    if not isinstance(pos_other, list):
        pos_other = [pos_other]

    N = len(t_other)
    if N == 0 or len(pos_other) == 0:
        return []
    t_other_sec = np.array(t_other, dtype=np.float64) * 1e-9
    pos_other = np.array(pos_other, dtype=np.float64)

    # Pad velocity if needed
    if len(vel_other) < N:
        vel_other = list(vel_other)
        for i in range(len(vel_other), N - 1):
            vel_other.append(np.zeros(3))
        # Estimate last velocity if possible
        if N > 1:
            dt_ = t_other_sec[-1] - t_other_sec[-2]
            if dt_ > 0:
                last_vel = (pos_other[-1] - pos_other[-2]) / dt_
            else:
                last_vel = np.zeros(3)
        else:
            last_vel = np.zeros(3)
        vel_other.append(last_vel)
    vel_other = np.array(vel_other, dtype=np.float64)

    # Pad acceleration if needed
    if len(acc_other) < len(vel_other):
        acc_other = list(acc_other)
        for i in range(len(acc_other), len(vel_other) - 1):
            acc_other.append(np.zeros(3))
        # Estimate last acceleration if possible
        if len(vel_other) > 1:
            dt_ = t_other_sec[-1] - t_other_sec[-2]
            if dt_ > 0:
                last_acc = (vel_other[-1] - vel_other[-2]) / dt_
            else:
                last_acc = np.zeros(3)
        else:
            last_acc = np.zeros(3)
        acc_other.append(last_acc)
    acc_other = np.array(acc_other, dtype=np.float64)

    # Query times
    t_query = t0 * 1e-9 + dt * np.arange(Nx)
    result = np.zeros((Nx, 3))

    for i, tq in enumerate(t_query):
        if N == 1 or tq >= t_other_sec[-1]:
            # Extrapolate from last point
            dt_extrap = tq - t_other_sec[-1]
            p = pos_other[-1]
            v = vel_other[-1]
            a = acc_other[-1]
            result[i] = p + v * dt_extrap + 0.5 * a * dt_extrap**2
        else:
            # Interpolate
            for j in range(N - 1):
                if t_other_sec[j] <= tq <= t_other_sec[j + 1]:
                    alpha = (tq - t_other_sec[j]) / (t_other_sec[j + 1] - t_other_sec[j])
                    result[i] = (1 - alpha) * pos_other[j] + alpha * pos_other[j + 1]
                    break

    return result