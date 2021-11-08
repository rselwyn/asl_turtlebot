import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        traj_smoothed (np.array [N,7]): Smoothed trajectory
        t_smoothed (np.array [N]): Associated trajectory times
    Hint: Use splrep and splev from scipy.interpolate
    """
    ########## Code starts here ##########
    path = np.array(path)

    t_nominal = np.zeros(path.shape[0])
    t_nominal[1:] = np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=-1)/V_des)

    tck, _ = scipy.interpolate.splprep([path[:,0], path[:,1]], u=t_nominal, s=alpha)

    t_smoothed = np.linspace(0, t_nominal[-1], int(t_nominal[-1]/dt))

    x_d, y_d = scipy.interpolate.splev(t_smoothed, tck)
    xd_d, yd_d = scipy.interpolate.splev(t_smoothed, tck, der=1)
    xdd_d, ydd_d = scipy.interpolate.splev(t_smoothed, tck, der=2)

    theta_d = np.zeros(t_smoothed.shape)
    theta_d[:-1] = np.arctan2(np.diff(y_d), np.diff(x_d))
    theta_d[-1] = theta_d[-2]

    traj_smoothed = np.stack([x_d, y_d, theta_d, xd_d, yd_d, xdd_d, ydd_d]).transpose()
    
    ########## Code ends here ##########

    return traj_smoothed, t_smoothed
