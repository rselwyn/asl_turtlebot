#!/usr/bin/env python
# update shebang

import tf
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point
from asl_turtlebot.msg import DetectedObjectList
from std_msgs.msg import String, Header
from tf.transformations import quaternion_from_euler
from utils.grids import StochOccupancyGrid2D

class SetTarget():
    def __int__(self):
        self.seen = []
        self.num_seen = 0
        self.threshold = .5
        self.trans_listener = tf.TransformListener()
        rospy.Subscriber("/detector/all", DetectedObjectList, self.update_seen_callback)
        # rospy.Subscriber("/current_position", Pose2D, queue_size=10)

    # euclidean distance between a pose and a DetectedObject
    def euclidean_distance(self, pose1, pose2):
        return (pose1.position.x - pose2.position.x) ** 2 + (pose1.position.y - pose2.position.y) ** 2

    def update_seen_callback(self, data):
        # These lines explain to the TA which objects we've seen by logging one whenever we do.
        # TODO: expand logging

        # list object if not already seen

        theta = (data.thetaleft + data.thetaright) / 2
        pose = Pose(Point(data.x, data.y, 0), quaternion_from_euler(0, 0, theta))
        world_pose = self.trans_listener.transformPose("/map", PoseStamped(Header(frame_id="/tf"), pose))
        data.world_pose = world_pose
        
        # maybe can optimize with some clever data structures
        filtered = [el for el in self.seen if el.name == world_pose.name]
        seen_sorted = sorted(filtered, 
            lambda key : self.euclidean_distance(key, world_pose)
        )

        if len(seen_sorted) == 0 or self.euclidean_distance(seen_sorted[0], data) > self.threshold:
            data.pose = world_pose
            self.seen.append(data)
            rospy.loginfo(f'#{len(self.seen)}: New object detected: {data.name}')
        elif len(seen_sorted) and self.euclidean_distance(seen_sorted[0], data) < self.threshold:
            self.seen[self.seen.index(seen_sorted[0])] = data
            

    def talker(self):
        rospy.init_node('talker', anonymous=True)

        pub = rospy.Publisher('/rescue', DetectedObjectList, queue_size=10)

        rate = rospy.get_param('~rate', 1)
        ros_rate = rospy.Rate(rate)

        rospy.loginfo('Starting ROS node talker...')
        
        while not rospy.is_shutdown():
            # inp = input("Which objects would you like to move to (space separated)? ").lower()   
            # targets = [int(el) for el in inp.split()]
            # detected_objects = DetectedObjectList()

            # for target in targets:
            #     detected_objects.objects.append(target.name)
            #     detected_objects.ob_msgs.append(target)

            # pub.publish(detected_objects)
            ros_rate.sleep()
    

if __name__ == '__main__':
    try:
        st = SetTarget()
        st.talker()
    except rospy.ROSInterruptException:
        pass