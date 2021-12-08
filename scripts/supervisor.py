#!/usr/bin/env python3

from enum import Enum

import rospy
import time
from asl_turtlebot.msg import DetectedObject
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float32MultiArray, String
import tf
from tf.transformations import quaternion_from_euler
import numpy as np
from utils.utils import wrapToPi
from nav_params import nav_params, POS_EPS, THETA_EPS
from asl_turtlebot.msg import DetectedObjectList

from std_msgs.msg import Header

class Mode(Enum):
    """State machine modes. Feel free to change."""
    IDLE = 1 # stationary
    EXPLORE = 2 # exploring the map
    RESCUE = 3 # rescuing specified objects
    STOP_SIGN = 4 # waiting at a stop sign
    LISTEN = 5 # listening for objects to rescue

# Helper transform functions

def pose2d_to_pose(pose: Pose2D):

    return Pose(Point(pose.x,pose.y,0),Quaternion(*quaternion_from_euler(0,0,pose.theta)))

def pose_to_pose2d(pose: Pose):

    quaternion = (pose.orientation.x,
                  pose.orientation.y,
                  pose.orientation.z,
                  pose.orientation.w)
    return Pose2D(pose.position.x, pose.position.y, tf.transformations.euler_from_quaternion(quaternion)[2])

# All points in world frame
EXPLORATION_WAYPOINTS = [
    (3.512, 1.847, np.pi/2), # center right
    (3.4, 2.72, np.pi/2), # top right
    (2.39, 2.72, np.pi), # top middle
    (2.39, 1.85, -np.pi/2),
    (2.25, 1.5, -np.pi/2),
    (2.25, 0.3, -np.pi/2), # bottom middle
    (0.36, 0.37, np.pi), # bottom left
    (0.31, 1.54, np.pi/2), # center left
    (2.26, 1.54, 0), # center
    (2.39, 2.87, np.pi/2), # top middle
    (1.45, 2.87, np.pi),
    (0.65, 2.64, -2.94),
    (0.31, 2.28, -np.pi/2),
    (0.31, 1.54, -np.pi/2), # center left
    (2.26, 1.54, 0), # center
    (2.25, 0.3, -np.pi/2), # bottom middle
    (3.4, 0.3, 0), # bottom right
    (3.4, 1.8, np.pi/2) # center right
]

EXPLORATION_WAYPOINTS = [Pose2D(*x) for x in EXPLORATION_WAYPOINTS]

class Supervisor:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('turtlebot_supervisor', anonymous=True)

        # Current state
        self.x = 0
        self.y = 0
        self.theta = 0

        # Goal state
        self.x_g = 0
        self.y_g = 0
        self.theta_g = 0

        # Current mode
        self.mode = Mode.IDLE
        self.prev_mode = None  # For printing purposes
        self.mode_before_stop = None
        self.navigator_enabled = False

        # Waypoint counter
        self.waypoint_index = 0

        # rescue waypoints
        self.rescue_goals
        self.rescue_index = 0
        self.rescue_timer = None
        self.rescue_threshold = 5

        # Time to stop at a stop sign
        self.stop_time = 3
        self.stop_sign_start = None

        # Minimum distance from a stop sign to obey it
        self.stop_min_dist = 0.5

        ########## PUBLISHERS ##########

        # Command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)

        # Command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Publisher to send messages to other nodes
        self.msg_publisher = rospy.Publisher('/supervisor', String, queue_size=10)

        ########## SUBSCRIBERS ##########

        # Stop sign detector
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)

        # rescue info subscriber
        rospy.Subscriber('/rescue', DetectedObjectList, self.update_rescue_goals_callback)

        self.trans_listener = tf.TransformListener()


    ########## SUBSCRIBER CALLBACKS ##########

    def update_rescue_goals_callback(self, msg):

        start = DetectedObject()
        start.world_pose = Pose(Point(0, 0, 0), quaternion_from_euler(0, 0, 0))
        self.rescue_goals = [m.world_pose for m in msg.ob_msgs].extend([start])
        self.switch_mode(Mode.RESCUE)
        self.set_goal_pose[self.rescue_index]

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance

        # if close enough and in nav mode, stop
        if dist > 0 and dist < self.stop_min_dist and self.mode == Mode.EXPLORE:
            self.init_stop_sign()


    ########## STATE MACHINE ACTIONS ##########

    ########## Code starts here ##########
    # Feel free to change the code here. You may or may not find these functions
    # useful. There is no single "correct implementation".

    def pose_to_navigator(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(nav_g_msg)

    def set_goal_pose(self, pose: Pose2D, source_frame="/odom"):
        """ sets goal pose attributes from a Pose2D object, doing any neccesary frame transformations """

        start = rospy.get_rostime()
        while True:
            try:
                p = pose2d_to_pose(pose)
                p = self.trans_listener.transformPose(nav_params.localizer, PoseStamped(Header(frame_id=source_frame), p))
                p = pose_to_pose2d(p.pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("TransformListener error")
                if rospy.get_rostime() - start > 0.5:
                    rospy.logerr("Unable to listen to /tf")
                    return
            else:
                break

        self.x_g = p.x
        self.y_g = p.y
        self.theta_g = p.theta

    def stay_idle(self):
        """ sends zero velocity to stay put """
        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self, x, y, theta):
        """ checks if the robot is at a pose within some threshold """

        # rospy.loginfo(f"x_diff {abs(x - self.x)}")
        # rospy.loginfo(f"y_diff {abs(y - self.y)}")
        # rospy.loginfo(f"theta_diff {wrapToPi(abs(theta - self.theta))}")

        return abs(x - self.x) < POS_EPS and \
               abs(y - self.y) < POS_EPS and \
               abs(wrapToPi(theta - self.theta)) < THETA_EPS

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """
        self.stop_sign_start = rospy.get_rostime()
        self.mode_before_stop = self.mode
        self.switch_mode(Mode.STOP_SIGN)

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return self.mode == Mode.STOP_SIGN and \
               rospy.get_rostime() - self.stop_sign_start > rospy.Duration.from_sec(self.stop_time)

    def switch_mode(self, new_mode):
        """ switch mode and enable / disable navigator as needed """

        if new_mode == Mode.IDLE or new_mode == Mode.STOP_SIGN or new_mode == Mode.LISTEN:
            if self.navigator_enabled:
                self.msg_publisher.publish("disable navigator")
                self.navigator_enabled = False
        else:
            if not self.navigator_enabled:
                self.msg_publisher.publish("enable navigator")
                self.navigator_enabled = True

        if new_mode == Mode.EXPLORE:
            nav_params.localizer = "odom"

        if new_mode == Mode.RESCUE:
            nav_params.localizer = "map"

        self.mode = new_mode


    ########## Code ends here ##########


    ########## STATE MACHINE LOOP ##########

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        try:
            translation, rotation = self.trans_listener.lookupTransform(nav_params.localizer, '/base_footprint', rospy.Time(0))
            self.x, self.y = translation[0], translation[1]
            self.theta = tf.transformations.euler_from_quaternion(rotation)[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        # logs the current mode
        if self.prev_mode != self.mode:
            rospy.loginfo("Current mode: %s", self.mode)
            self.prev_mode = self.mode

        ########## Code starts here ##########
        # TODO: Currently the state machine will just go to the pose without stopping
        #       at the stop sign.

        if self.mode == Mode.IDLE:
            # Send zero velocity
            self.stay_idle()

        elif self.mode == Mode.RESCUE:

            if self.close_to(self.x_g, self.y_g, self.theta_g):
                # if we've reached home, then idle--we're done!
                if (self.x_g, self.y_g, self.theta_g) == (0, 0, 0):
                    self.switch_mode(Mode.IDLE)

                # once we've reached, start waiting
                elif self.rescue_timer is None:
                    self.rescue_timer = time.time()

                # once we've waited long enough, stop waiting and go to next one.
                elif time.time() - self.rescue_timer > self.rescue_threshold:
                    self.rescue_timer = None
                    self.rescue_index += 1
                    self.set_goal_pose(self.rescue_goals[self.rescue_index])
                    self.pose_to_navigator()
            else:
                self.pose_to_navigator()

        elif self.mode == Mode.STOP_SIGN:
            # At a stop sign
            if self.has_stopped():
                self.switch_mode(self.mode_before_stop)
                self.mode_before_stop = None
            else:
                self.stay_idle()

        elif self.mode == Mode.EXPLORE:
            if self.close_to(self.x_g, self.y_g, self.theta_g):
                rospy.loginfo("CLOSE!")
                if self.waypoint_index < len(EXPLORATION_WAYPOINTS)-1:
                    self.waypoint_index += 1
                    self.set_goal_pose(EXPLORATION_WAYPOINTS[self.waypoint_index])
                    self.pose_to_navigator()
                else:
                    self.switch_mode(Mode.LISTEN)
            else:
                self.pose_to_navigator()
        
        elif self.mode == Mode.LISTEN:
            
            # TODO: Wait for list of items to rescue
            # If we have gotten a list, generate a set of waypoints and then change to mode pose
            self.stay_idle()

        else:
            raise Exception("This mode is not supported: {}".format(str(self.mode)))

    def start(self):
        self.switch_mode(Mode.EXPLORE)
        self.set_goal_pose(EXPLORATION_WAYPOINTS[self.waypoint_index])

        ############ Code ends here ############

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        self.start()
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
