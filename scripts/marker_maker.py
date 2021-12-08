#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from visualization_msgs.msg import Marker

vis_pub = rospy.Publisher('marker_topic', Marker, queue_size=10)

color_options = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0.5, 0.5, 0.5]]
marked_count = 0

def mark(x_position, y_position):
    
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time()
    marker.id = 0
    marker.type = 2
    
    marker.pose.position.x = data.x
    marker.pose.position.y = data.y
    
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    
    marker.color.a = 1.0
    marker.color.r = color_options[marked_count % len(color_options)][0]
    marker.color.g = color_options[marked_count % len(color_options)][1]
    marker.color.b = color_options[marked_count % len(color_options)][2]
    marked_count += 1
    
    vis_pub.publish(marker)
