"""Synthetic demo only. It never connects to a MANUS device."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from manus_ros2_msgs.msg import ManusGlove, ManusRawNode


class MockGloves(Node):
    def __init__(self):
        super().__init__('manus_mock_gloves')
        self.pubs = [self.create_publisher(ManusGlove, '/manus/demo_left', 10),
                     self.create_publisher(ManusGlove, '/manus/demo_right', 10)]
        self.start = time.monotonic()
        self.create_timer(0.05, self.tick)
        self.get_logger().warn('SYNTHETIC DATA: no glove or Windows Core is connected.')

    def tick(self):
        bend = 0.5 + 0.5*math.sin((time.monotonic()-self.start)*1.5)
        for pub, side, glove_id in zip(self.pubs, ('left', 'right'), (101, 102)):
            msg = ManusGlove(glove_id=glove_id, side=side)
            root = ManusRawNode(node_id=0, parent_node_id=0, chain_type='Hand', joint_type='Wrist')
            root.pose.orientation.w = 1.0
            msg.raw_nodes.append(root)
            for finger in range(5):
                parent = 0
                for joint in range(1, 5):
                    node_id = 1 + finger*4 + joint-1
                    node = ManusRawNode(node_id=node_id, parent_node_id=parent,
                                        chain_type=f'Finger{finger}', joint_type=f'Joint{joint}')
                    node.pose.position.x = joint*0.026*(1.0-0.30*bend*joint/4)
                    node.pose.position.y = (finger-2)*0.019
                    node.pose.position.z = -0.010*bend*joint*joint
                    node.pose.orientation.w = 1.0
                    msg.raw_nodes.append(node)
                    parent = node_id
            msg.raw_node_count = len(msg.raw_nodes)
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockGloves()
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
