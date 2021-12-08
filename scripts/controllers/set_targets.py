#!/usr/bin/env python
# update shebang

import tf
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
from std_msgs.msg import String, Header
from utils.grids import StochOccupancyGrid2D

class SetTarget():
    def __int__(self):
        self.seen = {}
        self.num_seen = 0
        self.trans_listener = tf.TransformListener()
        rospy.Subscriber("/detector/all", DetectedObject, self.update_seen_callback)
        rospy.Subscriber("/current_position", Pose2D, queue_size=10)

    def update_seen_callback(self, data):
        # These lines explain to the TA which objects we've seen by logging one whenever we do.
        # TODO: expand logging

        # list object if not already seen
        pose = self.trans_listener.transformPose("/map", PoseStamped(Header(frame_id="/odom"), pose))
        if data.id not in self.seen:
            self.
            rospy.loginfo(f"Object #{len(self.seen)}We've just observed a {data.name} at (") 

        # blindly update to only keep latest observed class member
        self.seen[data.id]
        


    def talker(self):
        rospy.init_node('talker', anonymous=True)

        pub = rospy.Publisher('/rescue', String, queue_size=10)

        rate = rospy.get_param('~rate', 1)
        ros_rate = rospy.Rate(rate)

        rospy.loginfo('Starting ROS node talker...')
        
        # dict to house information about observed objects

        while not rospy.is_shutdown():
            inp = input("What object would you like to move to (comma separated)? ").lower()   
                

            pub.publish(inp)
            ros_rate.sleep()
    

if __name__ == '__main__':
    try:
        st = SetTarget()
        st.talker()
    except rospy.ROSInterruptException:
        pass