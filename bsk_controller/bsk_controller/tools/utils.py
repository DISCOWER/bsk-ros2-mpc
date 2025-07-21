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

def MRP2quat(sigma):
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
    denom = 1 + s2
    qw = (1 - s2) / denom
    qx = 2 * sigma[0] / denom
    qy = 2 * sigma[1] / denom
    qz = 2 * sigma[2] / denom
    return np.hstack((qw, qx, qy, qz))