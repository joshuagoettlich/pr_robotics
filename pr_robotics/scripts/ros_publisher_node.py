from controll_class import * 
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import threading

class DynamixelROSInterface:
    def __init__(self, dxl_ids, joint_names, position_limits=None):
        self.dxl_ids = dxl_ids
        self.joint_names = joint_names
        self.controller = DynamixelController(dxl_ids, position_limits=position_limits)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.joint_sub = rospy.Subscriber('/desired_joint_state', JointState, self.joint_command_cb)
        self.current_positions = [0] * len(dxl_ids)
        self.command_positions = self.controller.read_positions()

        self.lock = threading.Lock()
        self.target_positions = self.controller.read_positions()


        # Use a separate thread to publish positions
        self.running = True
        self.pub_thread = threading.Thread(target=self.publish_loop)
        self.pub_thread.start()

    def joint_command_cb(self, msg):
        if len(msg.position) != len(self.dxl_ids):
            rospy.logwarn("Received joint command of wrong length.")
            return
        with self.lock:
            self.target_positions = list(msg.position)
        # Convert from radians to Dynamixel units (you can adjust the scale based on your model)
        self.command_positions = [int(pos) for pos in self.target_positions]

    def publish_loop(self):
        hz = 60
        rate = rospy.Rate(hz)  # 10 Hz
        while not rospy.is_shutdown() and self.running:
            print(hz)
            dxl_positions = self.controller.read_positions()

            joint_msg = JointState()
            joint_msg.header = Header()
            joint_msg.header.stamp = rospy.Time.now()
            joint_msg.name = self.joint_names
            joint_msg.position = [pos for pos in dxl_positions]  # Assuming these are already in rad/deg, otherwise convert
            self.joint_pub.publish(joint_msg)
            self.controller.write_positions(self.command_positions)
            self.current_positions = dxl_positions

            rate.sleep()

    def shutdown(self):
        self.running = False
        self.pub_thread.join()
        self.controller.disable_torque_and_close()

    

if __name__ == '__main__':
    try:
        rospy.init_node('dynamixel_joint_interface')
        dxl_ids = [1, 2, 3]
        joint_names = ['joint1', 'joint2', 'joint3']
        limits = [(1200, 2800), (800, 1600), (2800, 3100)]

        node = DynamixelROSInterface(dxl_ids, joint_names, position_limits=limits)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.shutdown()

