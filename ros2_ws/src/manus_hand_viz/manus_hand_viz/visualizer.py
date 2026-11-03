"""Display each glove in a separate visual-only slot; no physical tracking implied."""
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from manus_ros2_msgs.msg import ManusGlove


class HandVisualizer(Node):
    def __init__(self):
        super().__init__('manus_hand_viz')
        self.declare_parameter('frame_id', 'manus_view')
        self.declare_parameter('slot_spacing', 0.30)
        self.declare_parameter('topic_prefix', '/manus')
        self.frame_id = self.get_parameter('frame_id').value
        self.spacing = float(self.get_parameter('slot_spacing').value)
        self.prefix = self.get_parameter('topic_prefix').value.rstrip('/')
        self.pub = self.create_publisher(MarkerArray, '/manus_viz/markers', 10)
        self.subs = {}
        self.create_timer(1.0, self.discover)
        self.get_logger().info('Waiting for MANUS topics; visualization is wrist-relative.')

    def discover(self):
        for topic, types in self.get_topic_names_and_types():
            if ('manus_ros2_msgs/msg/ManusGlove' not in types or topic in self.subs or
                    not (topic.startswith('/manus_glove_') or topic.startswith(self.prefix + '/'))):
                continue
            slot = len(self.subs)
            self.subs[topic] = self.create_subscription(
                ManusGlove, topic, lambda msg, t=topic, s=slot: self.callback(msg, t, s), 10)
            self.get_logger().info(f'Discovered {topic}; display slot {slot}')

    def callback(self, msg, topic, slot):
        nodes = {n.node_id: n for n in msg.raw_nodes
                 if all(math.isfinite(v) for v in (n.pose.position.x, n.pose.position.y, n.pose.position.z))}
        if not nodes:
            return
        # Prefer the declared root, not array ordering. Raw SDK coordinates are world-space
        # with hand motion disabled; subtracting the root is for display only.
        roots = [n for n in nodes.values() if n.parent_node_id == n.node_id or n.parent_node_id not in nodes]
        root = roots[0] if roots else next(iter(nodes.values()))
        p0 = root.pose.position
        def point(n):
            p = n.pose.position
            return Point(x=p.x-p0.x, y=p.y-p0.y + slot*self.spacing, z=p.z-p0.z)
        now = self.get_clock().now().to_msg()
        def marker(mid, kind):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.ns = topic
            m.id = mid
            m.type = kind
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.lifetime = Duration(seconds=0.8).to_msg()
            m.color.a = 1.0
            m.color.r, m.color.g, m.color.b = ((0.22, 0.75, 1.0) if msg.side.lower() == 'left'
                                                else (1.0, 0.62, 0.24))
            return m
        dots = marker(0, Marker.SPHERE_LIST)
        dots.scale.x = dots.scale.y = dots.scale.z = 0.009
        dots.points = [point(n) for n in nodes.values()]
        lines = marker(1, Marker.LINE_LIST)
        lines.scale.x = 0.004
        for n in nodes.values():
            if n.parent_node_id in nodes and n.parent_node_id != n.node_id:
                lines.points.extend((point(nodes[n.parent_node_id]), point(n)))
        label = marker(2, Marker.TEXT_VIEW_FACING)
        label.pose.position = Point(x=0.0, y=slot*self.spacing, z=0.065)
        label.scale.z = 0.035
        label.text = f'{msg.side}  {msg.glove_id & 0xffffffff:08X}\n{topic}'
        self.pub.publish(MarkerArray(markers=[dots, lines, label]))


def main(args=None):
    rclpy.init(args=args)
    node = HandVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass
