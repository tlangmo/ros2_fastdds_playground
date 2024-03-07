import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from playground_interfaces.msg import StringWithHeader
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
"""
The qos_profile_default consists of several QoS policies, including:

Reliability: Controls whether the communication protocol guarantees delivery of messages. The default setting is typically RELIABLE, meaning it will try to ensure that all messages arrive.
Durability: Determines if messages should be stored on the publisher side to be delivered to late-joining subscribers. The default is usually VOLATILE, indicating that only subscribers present when the message is published will receive it.
History: Defines how many messages to keep in the history buffer. The default setting often keeps only the last message (KEEP_LAST) with a depth of 1.
Depth: This parameter is related to the History policy and specifies the size of the history buffer. A depth of 1 means only the most recent message is stored.
Deadline: Specifies the maximum acceptable delay between messages. By default, there is no deadline.
Liveliness: Indicates the mechanism used to determine if a publisher is "alive." The default setting typically assumes that publishers are always alive.
Lease Duration: Relates to the Liveliness policy and defines how long the system should consider a publisher alive without receiving any signals or messages from it. By default, the lease duration is infinite.

QoSProfile(history=HistoryPolicy.KEEP_LAST,
           depth=1,
           reliability=ReliabilityPolicy.RELIABLE,
           durability=DurabilityPolicy.VOLATILE,
           lifespan=0 nanoseconds, deadline=0 nanoseconds,
           liveliness=LivelinessPolicy.SYSTEM_DEFAULT,
           liveliness_lease_duration=0 nanoseconds,
           avoid_ros_namespace_conventions=False)
"""


def create_qos_profiles() -> dict[str, QoSProfile]:
    return {"reliable": QoSProfile(depth=10),
            "best_effort": QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            "transient_local_d3": QoSProfile(depth=3, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            "transient_local_d1": QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            "keep_all": QoSProfile(history=HistoryPolicy.KEEP_ALL)
            }


class QoSPubNode(Node):
    """Node which publishes topics which a variety of Qos profiles."""

    def __init__(self):
        super().__init__('QoSPubNode')
        self.declare_parameter('period_sec', 10)
        # Get the 'fps' parameter value
        period_sec = self.get_parameter(
            'period_sec').get_parameter_value().integer_value

        self.qos_publishers = {name: self.create_publisher(
            StringWithHeader, f"qos/{name}", profile) for name, profile in create_qos_profiles().items()}
        self.timer = self.create_timer(period_sec, self.publish_topics)
        self.counter = 0
        self.publish_topics()

    def publish_topics(self):
        header = Header()
        now = self.get_clock().now()  # Get current time in ROS clock
        header.stamp = Time(sec=int(now.seconds_nanoseconds()[
                            0]), nanosec=now.seconds_nanoseconds()[1])
        # Example frame_id, adjust as needed
        header.frame_id = f"{self.counter}"

        for name, publisher in self.qos_publishers.items():
            msg = StringWithHeader()
            msg.header = header
            msg.message = f'This message has qos profile \"{name}\"'
            publisher.publish(msg)
            self.get_logger().info(f'Publishing: "{msg}"')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = QoSPubNode()
    try:
        # Spin the node so it can perform its work (e.g., handling subscriptions, timers)
        rclpy.spin(node)
    except KeyboardInterrupt as ki:
        # Destroy the node explicitly - should do this in every Exception type except for
        # BaseException.
        node.destroy_node()
    finally:
        print('Node has been shut down gracefully')


if __name__ == '__main__':
    main()
