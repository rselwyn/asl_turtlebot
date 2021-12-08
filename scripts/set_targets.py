#!/usr/bin/env python3
# update shebang

import tf
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseArray
from asl_turtlebot.msg import DetectedObjectList, DetectedObject
from std_msgs.msg import String, Header
from tf.transformations import quaternion_from_euler
from utils.grids import StochOccupancyGrid2D
from visualization_msgs.msg import Marker

class SetTarget():
    def __init__(self):

        rospy.init_node('talker', anonymous=True)

        self.seen = []
        self.objs = []
        self.num_seen = 0
        self.threshold = 0.5
        self.trans_listener = tf.TransformListener()
        self.s = rospy.Subscriber("/detector/all", DetectedObject, self.update_seen_callback)
        self.should_output = False
        # rospy.Subscriber("/current_position", Pose2D, queue_size=10)
        self.marked_count = 0

        self.vis_pub = rospy.Publisher('marker_topic', Marker, queue_size=10)


        rospy.Subscriber("/supervisor", String, self.supervisor_callback)

    def supervisor_callback(self, string):
        rospy.loginfo("out")
        if string.data == "stopsign":
            rospy.loginfo("hi")
            self.should_output = True

    def mark(self, x_position, y_position):

        color_options = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0.5, 0.5, 0.5]]

        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()
        marker.id = 0
        marker.type = 2
        
        marker.pose.position.x = x_position
        marker.pose.position.y = y_position
        
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        marker.color.a = 1.0
        marker.color.r = color_options[self.marked_count % len(color_options)][0]
        marker.color.g = color_options[self.marked_count % len(color_options)][1]
        marker.color.b = color_options[self.marked_count % len(color_options)][2]
        self.marked_count += 1
        
        self.vis_pub.publish(marker)

        
    # euclidean distance between a pose and a DetectedObject
    def euclidean_distance(self, pose1, pose2):
        return (pose1.pose.position.x - pose2.pose.position.x) ** 2 + (pose1.pose.position.y - pose2.pose.position.y) ** 2

    def update_seen_callback(self, data):
        # These lines explain to the TA which objects we've seen by logging one whenever we do.
        # TODO: expand logging

        # list object if not already seen

        # rospy.loginfo(data)

        # x = (data.corners[1] + data.corners[3])/2
        # y = (data.corners[0] + data.corners[2])/2

        # theta = (data.thetaleft + data.thetaright) / 2
        # pose = Pose(Point(x, y, 0), Quaternion(*quaternion_from_euler(0, 0, theta)))
        # world_pose = self.trans_listener.transformPose("/map", PoseStamped(Header(frame_id="/base_camera"), pose))
        
        try:
            translation, rotation = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            x, y = translation[0], translation[1]
            theta = tf.transformations.euler_from_quaternion(rotation)[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        if len(self.seen)==0 or np.sqrt((x-sum([i[1][0] for i in self.seen])/len(self.seen))**2 + (y-sum([i[1][1] for i in self.seen])/len(self.seen))**2) < self.threshold:
            self.seen.append((data.name, (x,y,theta)))

        
        # # maybe can optimize with some clever data structures
        # filtered = [obj for obj in self.seen if obj[0].name == data.name]
        # seen_sorted = sorted(filtered, 
        #     key=lambda obj : self.euclidean_distance(obj[1], world_pose)
        # )

        # if len(seen_sorted) > 0:
        #     rospy.loginfo(f"dist: {self.euclidean_distance(seen_sorted[0][1], world_pose)}")
        
        # if len(seen_sorted) == 0 or self.euclidean_distance(seen_sorted[0][1], world_pose) > self.threshold:
        #     self.seen.append((data, world_pose))
        #     rospy.loginfo(f'#{len(self.seen)}: New object detected: {data.name}')
        # elif len(seen_sorted) and self.euclidean_distance(seen_sorted[0][1], world_pose) < self.threshold:
        #     # might be buggy-- may need to average world pose and existing, stored pose
        #     self.seen[self.seen.index(seen_sorted[0])] = (data, world_pose)
            

    def talker(self):

        pub = rospy.Publisher('/rescue', PoseArray, queue_size=10)

        # rate = rospy.get_param('~rate', 1)
        ros_rate = rospy.Rate(1)

        rospy.loginfo('Starting ROS node talker...')

        # rospy.spin()
        
        
        while not rospy.is_shutdown():

            try:
                translation, rotation = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                x, y = translation[0], translation[1]
                theta = tf.transformations.euler_from_quaternion(rotation)[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            else:
                if len(self.seen) > 0 and np.sqrt((x-sum([i[1][0] for i in self.seen])/len(self.seen))**2 + (y-sum([i[1][1] for i in self.seen])/len(self.seen))**2) > self.threshold:
                    x_bar = sum([i[1][0] for i in self.seen])/len(self.seen)
                    y_bar = sum([i[1][1] for i in self.seen])/len(self.seen)
                    self.objs.append((self.seen[0][0],(x_bar, y_bar, sum([i[1][2] for i in self.seen])/len(self.seen))))
                    rospy.loginfo(f'found object #{len(self.objs)-1} {self.objs[-1]}')
                    self.seen = []
                    self.mark(x_bar, y_bar)
            
            if self.should_output:

                rospy.loginfo("sdd")

                inp = input("Which objects would you like to move to (space separated)? ").lower()   
                targets = [int(el) for el in inp.split()]

                detected_objects = [x for i,x in enumerate(self.objs) if i in targets]

                array = PoseArray()
                array.header = "map"
                array.poses = [Pose(Point(*x[1]), Quaternion()) for x in detected_objects]

                # for target in targets:
                #     detected_objects.objects.append(target.name)
                #     detected_objects.ob_msgs.append(target)

                pub.publish(detected_objects)

                self.should_output = False


            ros_rate.sleep()
    

if __name__ == '__main__':
    try:
        st = SetTarget()
        st.talker()
    except rospy.ROSInterruptException:
        pass
