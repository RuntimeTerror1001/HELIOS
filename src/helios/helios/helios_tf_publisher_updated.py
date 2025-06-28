from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from rclpy.node import Node
import rclpy
from scipy.spatial.transform import Rotation as R
import math

class HeliosTFPublisherUpdated(Node):
    def __init__(self):
        super().__init__('helios_tf_publisher_updated')
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.joint_positions = {}
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_rotor_tfs)  # 10 Hz

        self.publish_static_tfs()

    def joint_state_callback(self, msg):
        self.joint_positions = dict(zip(msg.name, msg.position))

    def publish_static_tfs(self):
        # Camera TF
        t_camera = TransformStamped()
        t_camera.header.stamp = self.get_clock().now().to_msg()
        t_camera.header.frame_id = 'base_link'
        t_camera.child_frame_id = 'camera_front'
        t_camera.transform.translation.x = 0.192
        t_camera.transform.translation.y = -0.015
        t_camera.transform.translation.z = -0.023
        q = R.from_euler('xyz', [0, 0, 0.174533]).as_quat()
        t_camera.transform.rotation.x = q[0]
        t_camera.transform.rotation.y = q[1]
        t_camera.transform.rotation.z = q[2]
        t_camera.transform.rotation.w = q[3]

        # Lidar TF
        t_lidar = TransformStamped()
        t_lidar.header.stamp = self.get_clock().now().to_msg()
        t_lidar.header.frame_id = 'base_link'
        t_lidar.child_frame_id = 'front_laser'
        t_lidar.transform.translation.x = 0.0
        t_lidar.transform.translation.y = 0.0
        t_lidar.transform.translation.z = 0.05
        t_lidar.transform.rotation.x = 0.0
        t_lidar.transform.rotation.y = 0.0
        t_lidar.transform.rotation.z = 0.0
        t_lidar.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform([t_camera, t_lidar])

    def publish_rotor_tfs(self):
        rotor_config = {
            "rotor_0_joint": [0.2263, -0.2263, 0.038],
            "rotor_1_joint": [-0.2263, 0.2263, 0.038],
            "rotor_2_joint": [0.2263, 0.2263, 0.038],
            "rotor_3_joint": [-0.2263, -0.2263, 0.038],
        }

        for joint, (x, y, z) in rotor_config.items():
            angle = self.joint_positions.get(joint, 0.0)
            q = R.from_euler('xyz', [0, 0, angle]).as_quat()
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = joint.replace("_joint", "")
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = HeliosTFPublisherUpdated()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
