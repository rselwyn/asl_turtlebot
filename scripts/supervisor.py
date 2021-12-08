#!/usr/bin/env python3

from enum import Enum

import rospy
from asl_turtlebot.msg import DetectedObject
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float32MultiArray, String
import tf
from tf.transformations import quaternion_from_euler
import numpy as np
from utils.utils import wrapToPi

from std_msgs.msg import Header

class Mode(Enum):
    """State machine modes. Feel free to change."""
    IDLE = 1 # stationary
    EXPLORE = 2 # exploring the map
    RESCUE = 3 # rescuing specified objects
    STOP_SIGN = 4 # waiting at a stop sign
    LISTEN = 5 # listening for objects to rescue


class SupervisorParams:

    def __init__(self, verbose=False):
        # If sim is True (i.e. using gazebo), we want to subscribe to
        # /gazebo/model_states. Otherwise, we will use a TF lookup.
        self.use_gazebo = False#rospy.get_param("sim")

        # How is nav_cmd being decided -- human manually setting it, or rviz
        self.rviz = rospy.get_param("rviz")

        # If using gmapping, we will have a map frame. Otherwise, it will be odom frame.
        self.mapping = rospy.get_param("map")

        # Threshold at which we consider the robot at a location
        self.pos_eps = rospy.get_param("~pos_eps", 0.02)
        self.theta_eps = rospy.get_param("~theta_eps", 0.05)

        # Time to stop at a stop sign
        self.stop_time = rospy.get_param("~stop_time", 3.)
        self.stop_sign_start = None

        # Minimum distance from a stop sign to obey it
        self.stop_min_dist = rospy.get_param("~stop_min_dist", 0.5)

        if verbose:
            print("SupervisorParams:")
            print("    use_gazebo = {}".format(self.use_gazebo))
            print("    rviz = {}".format(self.rviz))
            print("    mapping = {}".format(self.mapping))
            print("    pos_eps, theta_eps = {}, {}".format(self.pos_eps, self.theta_eps))
            print("    stop_time, stop_min_dist = {}, {}".format(self.stop_time, self.stop_min_dist))

def pose2d_to_pose(pose: Pose2D):

    return Pose(Point(pose.x,pose.y,0),Quaternion(*quaternion_from_euler(0,0,pose.theta)))

def pose_to_pose2d(pose: Pose):
    quaternion = (pose.orientation.x,
                  pose.orientation.y,
                  pose.orientation.z,
                  pose.orientation.w)
    return Pose2D(pose.position.x, pose.position.y, tf.transformations.euler_from_quaternion(quaternion)[2])

# odom frame
EXPLORATION_WAYPOINTS = [
    (3.512, 1.847, np.pi/2),
    (3.4, 2.7, np.pi/2),
    (2.5, 2.7, np.pi),
    (2.5, 1.5, -np.pi/2),
    
]

EXPLORATION_WAYPOINTS = [Pose2D(*x) for x in EXPLORATION_WAYPOINTS]

class Supervisor:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        self.params = SupervisorParams(verbose=True)

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

        self.waypoint_index = 0

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

        # If using gazebo, we have access to perfect state
        if self.params.use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.trans_listener = tf.TransformListener()

        # If using rviz, we can subscribe to nav goal click
        # if self.params.rviz:
        #     rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        # else:
        #     self.x_g, self.y_g, self.theta_g = 1.5, -4., 0.
        #     self.set_mode(Mode.EXPLORE)
        

    ########## SUBSCRIBER CALLBACKS ##########

    def gazebo_callback(self, msg):
        if "turtlebot3_burger" not in msg.name:
            return

        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """

        # origin_frame = "/map" if self.params.mapping else "/odom"
        # print("Rviz command received!")

        # try:
        #     nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
        #     self.x_g = nav_pose_origin.pose.position.x
        #     self.y_g = nav_pose_origin.pose.position.y
        #     quaternion = (nav_pose_origin.pose.orientation.x,
        #                   nav_pose_origin.pose.orientation.y,
        #                   nav_pose_origin.pose.orientation.z,
        #                   nav_pose_origin.pose.orientation.w)
        #     euler = tf.transformations.euler_from_quaternion(quaternion)
        #     self.theta_g = euler[2]
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     pass

        # Begin moving through hardcoded waypoints once we command any pose through rviz
        self.set_mode(Mode.EXPLORE)

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance

        # if close enough and in nav mode, stop
        if dist > 0 and dist < self.params.stop_min_dist and self.mode == Mode.EXPLORE:
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

        while True:
            #TODO: Add timeout
            try:
                p = pose2d_to_pose(pose)
                p = self.trans_listener.transformPose("/map" if self.params.mapping else "/odom", PoseStamped(Header(frame_id=source_frame), p))
                p = pose_to_pose2d(p.pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # pose = Pose2D(x=self.x,y=self.y,theta=self.theta)
                rospy.logerr("TransformListener error")
                pass
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

        return abs(x - self.x) < self.params.pos_eps and \
               abs(y - self.y) < self.params.pos_eps and \
               abs(wrapToPi(theta - self.theta)) < self.params.theta_eps

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """
        self.stop_sign_start = rospy.get_rostime()
        self.mode_before_stop = self.mode
        self.set_mode(Mode.STOP_SIGN)

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return self.mode == Mode.STOP_SIGN and \
               rospy.get_rostime() - self.stop_sign_start > rospy.Duration.from_sec(self.params.stop_time)

    def set_mode(self, new_mode):
        """ set mode and enable / disable navigator as needed """

        if new_mode == Mode.IDLE or new_mode == Mode.STOP_SIGN or new_mode == Mode.LISTEN:
            if self.navigator_enabled:
                self.msg_publisher.publish("disable navigator")
                self.navigator_enabled = False
        else:
            if not self.navigator_enabled:
                self.msg_publisher.publish("enable navigator")
                self.navigator_enabled = True

        self.mode = new_mode


    ########## Code ends here ##########


    ########## STATE MACHINE LOOP ##########

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        if not self.params.use_gazebo:
            try:
                origin_frame = "/map" if self.params.mapping else "/odom"
                translation, rotation = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
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

            # TODO: Loop through rescue waypoints and pause as needed

            if self.close_to(self.x_g, self.y_g, self.theta_g):
                self.set_mode(Mode.IDLE)
            else:
                self.pose_to_navigator()

        elif self.mode == Mode.STOP_SIGN:
            # At a stop sign
            if self.has_stopped():
                self.set_mode(self.mode_before_stop)
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
                    self.set_mode(Mode.LISTEN)
            else:
                self.pose_to_navigator()
        
        elif self.mode == Mode.LISTEN:
            
            # TODO: Wait for list of items to rescue
            # If we have gotten a list, generate a set of waypoints and then change to mode pose
            self.stay_idle()

        else:
            raise Exception("This mode is not supported: {}".format(str(self.mode)))

    def start(self):
        self.set_mode(Mode.EXPLORE)
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
