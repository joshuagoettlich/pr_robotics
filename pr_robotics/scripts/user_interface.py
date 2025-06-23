#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

def main():
    rospy.init_node('joint_state_publisher')
    pub = rospy.Publisher('/desired_joint_state', JointState, queue_size=10)
    rospy.sleep(0.5)  # Give ROS time to set up the publisher

    print("Publishing joint positions. Press Ctrl+C to exit.")

    while not rospy.is_shutdown():
        try:
            joint1 = float(input("Enter position for joint1: "))
            joint2 = float(input("Enter position for joint2: "))
            joint3 = float(input("Enter position for joint3: "))
        except ValueError:
            print("Invalid input. Please enter numeric values.")
            continue
        except KeyboardInterrupt:
            print("\nExiting.")
            break

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['joint1', 'joint2', 'joint3']
        msg.position = [joint1, joint2, joint3]
        msg.velocity = []
        msg.effort = []

        pub.publish(msg)
        rospy.loginfo("Published: %s", msg.position)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
