import numpy as np

EPSILON_OMEGA = 1e-3

def compute_dynamics(xvec, u, dt, compute_jacobians=True):
    """
    Compute Turtlebot dynamics (unicycle model).

    Inputs:
                     xvec: np.array[3,] - Turtlebot state (x, y, theta).
                        u: np.array[2,] - Turtlebot controls (V, omega).
        compute_jacobians: bool         - compute Jacobians Gx, Gu if true.
    Outputs:
         g: np.array[3,]  - New state after applying u for dt seconds.
        Gx: np.array[3,3] - Jacobian of g with respect to xvec.
        Gu: np.array[3,2] - Jacobian of g with respect to u.
    """
    ########## Code starts here ##########
    # TODO: Compute g, Gx, Gu
    # HINT: To compute the new state g, you will need to integrate the dynamics of x, y, theta
    # HINT: Since theta is changing with time, try integrating x, y wrt d(theta) instead of dt by introducing om
    # HINT: When abs(om) < EPSILON_OMEGA, assume that the theta stays approximately constant ONLY for calculating the next x, y
    #       New theta should not be equal to theta. Jacobian with respect to om is not 0.

    theta = xvec[2]
    V, w = u
    
    if abs(w) < EPSILON_OMEGA:

        g = xvec[0:3] + np.array([
            V*dt*np.cos(theta),
            V*dt*np.sin(theta),
            w*dt,
        ])

        Gx = np.array([
            [1, 0, -V*dt*np.sin(theta)],
            [0, 1, V*dt*np.cos(theta)],
            [0, 0, 1],
        ])

        Gu = np.array([
            [dt*np.cos(theta), 0],
            [dt*np.sin(theta), 0],
            [0, dt],
        ])

    else:
        
        g = xvec[0:3] + np.array([
            V/w*(np.sin(theta + w*dt) - np.sin(theta)),
            -V/w*(np.cos(theta + w*dt) - np.cos(theta)),
            w*dt,
        ])

        Gx = np.array([
            [1, 0, (V*(np.cos(theta + dt*w) - np.cos(theta)))/w],
            [0, 1, (V*(np.sin(theta + dt*w) - np.sin(theta)))/w],
            [0, 0, 1],
        ])

        Gu = np.array([
            [(np.sin(theta + dt*w) - np.sin(theta))/w, (V*dt*np.cos(theta + dt*w))/w - (V*(np.sin(theta + dt*w) - np.sin(theta)))/w**2],
            [-(np.cos(theta + dt*w) - np.cos(theta))/w, (V*(np.cos(theta + dt*w) - np.cos(theta)))/w**2 + (V*dt*np.sin(theta + dt*w))/w],
            [0, dt],
        ])
    
    ########## Code ends here ##########

    if not compute_jacobians:
        return g

    return g, Gx, Gu

def transform_line_to_scanner_frame(line, x, tf_base_to_camera, compute_jacobian=True):
    """
    Given a single map line in the world frame, outputs the line parameters
    in the scanner frame so it can be associated with the lines extracted
    from the scanner measurements.

    Input:
                     line: np.array[2,] - map line (alpha, r) in world frame.
                        x: np.array[3,] - pose of base (x, y, theta) in world frame.
        tf_base_to_camera: np.array[3,] - pose of camera (x, y, theta) in base frame.
         compute_jacobian: bool         - compute Jacobian Hx if true.
    Outputs:
         h: np.array[2,]  - line parameters in the scanner (camera) frame.
        Hx: np.array[2,3] - Jacobian of h with respect to x.
    """
    alpha, r = line

    ########## Code starts here ##########
    # TODO: Compute h, Hx
    # HINT: Calculate the pose of the camera in the world frame (x_cam, y_cam, th_cam), a rotation matrix may be useful.
    # HINT: To compute line parameters in the camera frame h = (alpha_in_cam, r_in_cam), 
    #       draw a diagram with a line parameterized by (alpha,r) in the world frame and 
    #       a camera frame with origin at x_cam, y_cam rotated by th_cam wrt to the world frame
    # HINT: What is the projection of the camera location (x_cam, y_cam) on the line r? 
    # HINT: To find Hx, write h in terms of the pose of the base in world frame (x_base, y_base, th_base)

    R = lambda theta : np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)],
    ])

    t_camera_w = R(x[2]) @ tf_base_to_camera[0:2] + x[0:2]
    alpha_c = alpha - (x[2] + tf_base_to_camera[2])
    r_c = r - (np.cos(alpha)*t_camera_w[0] + np.sin(alpha)*t_camera_w[1])

    h = np.array([alpha_c, r_c])

    Hx = np.array([
        [0,0,-1],
        [-np.cos(alpha),-np.sin(alpha),-np.array([np.cos(alpha), np.sin(alpha)]).dot(R(x[2]+np.pi/2) @ tf_base_to_camera[0:2])]
    ])

    h, Hx = normalize_line_parameters(h, Hx)

    ########## Code ends here ##########

    if not compute_jacobian:
        return h

    return h, Hx


def normalize_line_parameters(h, Hx=None):
    """
    Ensures that r is positive and alpha is in the range [-pi, pi].

    Inputs:
         h: np.array[2,]  - line parameters (alpha, r).
        Hx: np.array[2,n] - Jacobian of line parameters with respect to x.
    Outputs:
         h: np.array[2,]  - normalized parameters.
        Hx: np.array[2,n] - Jacobian of normalized line parameters. Edited in place.
    """
    alpha, r = h
    if r < 0:
        alpha += np.pi
        r *= -1
        if Hx is not None:
            Hx[1,:] *= -1
    alpha = (alpha + np.pi) % (2*np.pi) - np.pi
    h = np.array([alpha, r])

    if Hx is not None:
        return h, Hx
    return h
