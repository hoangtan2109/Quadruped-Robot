#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from quadruped_control.msg import TaskSpace
from collections import deque

MAX_POINTS = 300

trails = {
    'FL': deque(maxlen=MAX_POINTS),
    'FR': deque(maxlen=MAX_POINTS),
    'RL': deque(maxlen=MAX_POINTS),
    'RR': deque(maxlen=MAX_POINTS),
}

colors = {
    'FL': (1.0, 0.0, 0.0),
    'FR': (0.0, 1.0, 0.0),
    'RL': (0.0, 0.0, 1.0),
    'RR': (1.0, 1.0, 0.0),
}

# Offset gốc chân so với base_link (lấy từ tf_echo)
offsets = {
    'FL': (0.117,  0.111, 0.0),
    'FR': (0.117, -0.111, 0.0),
    'RL': (-0.117,  0.111, 0.0),
    'RR': (-0.117, -0.111, 0.0),
}

pub = None

def callback(msg):
    feet = {
        'FL': msg.FL_foot,
        'FR': msg.FR_foot,
        'RL': msg.RL_foot,
        'RR': msg.RR_foot,
    }
    for leg, pt in feet.items():
        ox, oy, oz = offsets[leg]
        trails[leg].append((pt.x + ox, pt.y + oy, pt.z + oz))

    marker_array = MarkerArray()
    for idx, (leg, trail) in enumerate(trails.items()):
        if len(trail) < 2:
            continue
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = leg
        marker.id = idx
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.008
        r, g, b = colors[leg]
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0

        for (x, y, z) in trail:
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            marker.points.append(p)
        marker_array.markers.append(marker)

    pub.publish(marker_array)

def main():
    global pub
    rospy.init_node("foot_trajectory_visualizer")
    pub = rospy.Publisher("/foot_trajectory", MarkerArray, queue_size=10)
    rospy.Subscriber("/task_space_goals", TaskSpace, callback)
    rospy.loginfo("Foot trajectory visualizer started!")
    rospy.spin()

main()
