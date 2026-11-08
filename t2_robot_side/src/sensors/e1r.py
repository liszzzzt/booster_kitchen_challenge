from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class E1RMount:
    def __init__(self, node):
        self.broadcaster = StaticTransformBroadcaster(node)

        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = "trunk"
        transform.child_frame_id = "e1r_link"
        transform.transform.translation.x = 0.17
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.05
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(transform)
