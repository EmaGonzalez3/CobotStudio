#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

def main(args=None):
    rclpy.init(args=args)
    
    # Creamos un nodo de forma temporal
    node = Node('initial_pose_publisher_quick')
    
    # Creamos el publisher
    publisher = node.create_publisher(JointState, 'joint_states', 10)
    
    # Definimos el mensaje
    # Nombre de las variables articulares
    joint_names = [
        'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 
        'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
        'gripper_controller', 'gripper_base_to_gripper_left2',
        'gripper_left3_to_gripper_left1', 'gripper_base_to_gripper_right3',
        'gripper_base_to_gripper_right2', 'gripper_right3_to_gripper_right1'
    ]
    # Pose: q = 0 (posición home)
    positions = [0.0] * len(joint_names)

    msg = JointState()
    msg.name = joint_names
    msg.position = positions
    
    # Damos un pequeño margen para que el publisher se registre en la red ROS2.
    node.get_logger().info('Esperando para que el publisher se establezca...')
    time.sleep(1.0)
    
    # Publicamos el mensaje
    msg.header.stamp = node.get_clock().now().to_msg()
    publisher.publish(msg)
    node.get_logger().info('Estado inicial publicado: robot en posición home. Saliendo...')
    
    # Esperamos un instante para asegurar que el mensaje se envíe por la red
    time.sleep(0.5)

    # Limpiamos el nodo
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()