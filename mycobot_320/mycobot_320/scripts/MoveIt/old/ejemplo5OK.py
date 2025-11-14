#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

# Importa los mensajes y servicios necesarios
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.srv import GetInteractiveMarkers
from geometry_msgs.msg import Pose

class RvizGoalImpersonator(Node):
    def __init__(self):
        super().__init__("rviz_goal_impersonator_final")

        # --- CONFIGURACIÓN ---
        self.FEEDBACK_TOPIC = "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback"
        
        # !! IMPORTANTE !!
        # Reemplaza estas cadenas con los nombres EXACTOS que encontraste en la salida del `ros2 service call`
        self.MARKER_NAME = "EE:goal_link6"  # El nombre del `marker` principal
        self.CONTROL_NAME = "" # El nombre del `control` que encontraste dentro del marker

        # El frame de referencia. Usualmente el base_link del robot.
        self.FRAME_ID = "base"

        # La pose objetivo
        self.goal_pose = Pose()
        self.goal_pose.position.x = 0.3
        self.goal_pose.position.y = -0.1
        self.goal_pose.position.z = 0.25
        self.goal_pose.orientation.w = 1.0

        # --- LÓGICA ---
        # 1. Crear el Publisher
        self.feedback_publisher = self.create_publisher(InteractiveMarkerFeedback, self.FEEDBACK_TOPIC, 10)
        self.get_logger().info(f"Publicando feedback para el control '{self.CONTROL_NAME}'...")
        time.sleep(1.0) # Dar tiempo para que se establezca la conexión

        # 2. Construir y publicar el mensaje de feedback
        self.publish_feedback()

        # Apagamos
        time.sleep(1.0)
        rclpy.shutdown()

    def publish_feedback(self):
        feedback_msg = InteractiveMarkerFeedback()
        feedback_msg.header.frame_id = self.FRAME_ID
        feedback_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Usamos los nombres que hemos descubierto y configurado
        feedback_msg.marker_name = self.MARKER_NAME
        feedback_msg.control_name = self.CONTROL_NAME

        # El tipo de evento que simula que el usuario ha movido el control
        feedback_msg.event_type = InteractiveMarkerFeedback.POSE_UPDATE

        # Asignamos nuestra pose objetivo
        feedback_msg.pose = self.goal_pose

        # Publicar el feedback
        self.feedback_publisher.publish(feedback_msg)
        self.get_logger().info("¡Feedback de 'POSE_UPDATE' enviado! El robot naranja debería articularse ahora.")

def main():
    rclpy.init()
    impersonator_node = RvizGoalImpersonator()
    # No necesitamos spin, el nodo hace su trabajo y se apaga
    impersonator_node.destroy_node()

if __name__ == '__main__':
    main()