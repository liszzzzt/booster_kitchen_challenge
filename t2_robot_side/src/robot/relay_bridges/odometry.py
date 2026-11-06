from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdometryRelay:
    def __init__(self, node):
        self.publisher = node.create_publisher(
            Odometry,
            "/t2/odom",
            qos_profile_sensor_data,
        )
        self.transform_broadcaster = TransformBroadcaster(node)
        self.subscription = node.create_subscription(
            Odometry,
            "/odom",
            self._relay,
            qos_profile_sensor_data,
        )

    def _relay(self, msg):
        self.publisher.publish(msg)

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = msg.header.frame_id
        transform.child_frame_id = msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        # odom --> trunk
        self.transform_broadcaster.sendTransform(transform)
