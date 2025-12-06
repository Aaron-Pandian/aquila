def simulate_imu(state):
    """
    Placeholder IMU simulation.

    Parameters
    ----------
    state : dict
        Current true state.

    Returns
    -------
    dict
        IMU measurements (accelerometer, gyro).
    """
    # TODO: add realistic noise and bias
    return {
        "accel_mps2": [0.0, 0.0, 0.0],
        "gyro_rads": [0.0, 0.0, 0.0],
    }