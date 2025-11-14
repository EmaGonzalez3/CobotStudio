#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler

from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker

from geometry_msgs.msg import Pose


class InteractiveCubeDemo(Node):
    def __init__(self):
        super().__init__("interactive_cube_demo")

        # ROS 2 → requiere name y namespace
        self.server = InteractiveMarkerServer(
            "robtargets",              # nombre del servidor
            ""                         # namespace (string vacío si no usás uno)
        )

        # Crear cubo
        self.add_movable_cube(
            name="cubo1",
            pose=self.make_pose(0.5, 0.6, 0.2),
            scale=0.05
        )

        self.server.applyChanges()
        self.get_logger().info("Interactive cube ready. Abrí RViz → Add → InteractiveMarkers → robtargets")


    # ------------------------------------------------------------------
    # Crear un cubo movible
    # ------------------------------------------------------------------
    def add_movable_cube(self, name: str, pose: Pose, scale: float):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base"
        int_marker.name = name
        int_marker.description = name
        int_marker.pose = pose
        int_marker.scale = 1.0  # acá va el factor global del interactive marker

        # Control MOVE_3D
        control = InteractiveMarkerControl()
        control.name = "move_3d"
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        control.always_visible = True

        cube_marker = Marker()
        cube_marker.type = Marker.CUBE
        cube_marker.scale.x = scale
        cube_marker.scale.y = scale
        cube_marker.scale.z = scale
        cube_marker.color.r = 0.2
        cube_marker.color.g = 0.8
        cube_marker.color._
