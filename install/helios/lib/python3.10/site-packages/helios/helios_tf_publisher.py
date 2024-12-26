from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
import rclpy

class HeliosTFPublisher(Node):
    def __init__(self):
        super().__init__('helios_tf_publisher')
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_sensor_tfs)  # Broadcast every 100ms

    def publish_sensor_tfs(self):
        # Publish the transform for the camera
        t_camera = TransformStamped()
        t_camera.header.stamp = self.get_clock().now().to_msg()
        t_camera.header.frame_id = 'HELIOS/HELIOS/base_link'  # Corrected frame_id to match the output
        t_camera.child_frame_id = 'HELIOS/HELIOS/base_link/camera_front'  # Corrected child_frame_id

        # Set the translation (position of the camera relative to base_link)
        t_camera.transform.translation.x = 0.192
        t_camera.transform.translation.y = -0.015
        t_camera.transform.translation.z = -0.023

        # Set the rotation (orientation of the camera relative to base_link)
        t_camera.transform.rotation.x = 0.0
        t_camera.transform.rotation.y = 0.0
        t_camera.transform.rotation.z = 0.174533  # Roll
        t_camera.transform.rotation.w = 1.0  # Identity quaternion (no pitch/yaw)

        # Broadcast the camera transform
        self.broadcaster.sendTransform(t_camera)

        # Publish the transform for the lidar
        t_lidar = TransformStamped()
        t_lidar.header.stamp = self.get_clock().now().to_msg()
        t_lidar.header.frame_id = 'HELIOS/HELIOS/base_link'  # Corrected frame_id to match the output
        t_lidar.child_frame_id = 'HELIOS/HELIOS/base_link/front_laser'  # Corrected child_frame_id

        # Set the translation (position of the lidar relative to base_link)
        t_lidar.transform.translation.x = 0.0
        t_lidar.transform.translation.y = 0.0
        t_lidar.transform.translation.z = 0.05  # Height of lidar

        # Set the rotation (orientation of the lidar relative to base_link)
        t_lidar.transform.rotation.x = 0.0
        t_lidar.transform.rotation.y = 0.0
        t_lidar.transform.rotation.z = 0.0  # No rotation
        t_lidar.transform.rotation.w = 1.0  # Identity quaternion

        # Broadcast the lidar transform
        self.broadcaster.sendTransform(t_lidar)

def main(args=None):
    rclpy.init(args=args)
    node = HeliosTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
