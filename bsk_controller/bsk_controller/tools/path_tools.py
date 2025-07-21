import numpy as np

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
