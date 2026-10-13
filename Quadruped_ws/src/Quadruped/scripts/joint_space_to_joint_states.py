#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

joint_names = [
    "FR_theta1", "FR_theta2", "FR_theta3",
    "FL_theta1", "FL_theta2", "FL_theta3",
    "RR_theta1", "RR_theta2", "RR_theta3",
    "RL_theta1", "RL_theta2", "RL_theta3"
]

positions = [0.0] * 12
pub = None

topics = [
    "/quadruped_controller/FR_theta1/command",
    "/quadruped_controller/FR_theta2/command",
    "/quadruped_controller/FR_theta3/command",
    "/quadruped_controller/FL_theta1/command",
    "/quadruped_controller/FL_theta2/command",
    "/quadruped_controller/FL_theta3/command",
    "/quadruped_controller/RR_theta1/command",
    "/quadruped_controller/RR_theta2/command",
    "/quadruped_controller/RR_theta3/command",
    "/quadruped_controller/RL_theta1/command",
    "/quadruped_controller/RL_theta2/command",
    "/quadruped_controller/RL_theta3/command",
]

def make_callback(i):
    def callback(msg):
        positions[i] = msg.data
    return callback

def main():
    global pub
    rospy.init_node("joint_space_to_joint_states")
    pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
    for i, topic in enumerate(topics):
        rospy.Subscriber(topic, Float64, make_callback(i))
    
    rate = rospy.Rate(20)  # 20Hz
    while not rospy.is_shutdown():
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = joint_names
        js.position = positions[:]
        pub.publish(js)
        rate.sleep()

main()
