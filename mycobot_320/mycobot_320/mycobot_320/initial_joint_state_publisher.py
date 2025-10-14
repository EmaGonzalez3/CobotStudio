#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class InitialJointStatePublisher(Node):
    def __init__(self):
        super().__init__('initial_joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.5, self.publish_once)
        self.done = False

    def publish_once(self):
        if self.done:
            return
        joint_names = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6','gripper_controller','gripper_base_to_gripper_left2','gripper_left3_to_gripper_left1','gripper_base_to_gripper_right3','gripper_base_to_gripper_right2','gripper_right3_to_gripper_right1']
        q = [0.0] * len(joint_names)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = q

        self.publisher.publish(msg)
        self.get_logger().info('Publicado estado inicial en q = [0, 0, 0, 0, 0, 0]')
        self.done = True

def main(args=None):
    rclpy.init(args=args)
    node = InitialJointStatePublisher()
    rclpy.spin_once(node, timeout_sec=1.0)  # Solo una publicación rápida
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
