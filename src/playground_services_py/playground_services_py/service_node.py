import rclpy
from rclpy.node import Node
from time import perf_counter
import rclpy
import rclpy.qos
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from playground_interfaces.srv import RunFakeTask
from rclpy.task import Future
from rclpy.qos import qos_profile_services_default


async def sleep(node: Node, timeout_sec: float):
    """Ros2 compatible coroutine sleep function.

    It works similar to asyncio.sleep() but it is compatible with the Ros2 event loop.
    @see https://www.notion.so/aescape/Asynchronous-programming-with-ROS-2-Python-Api-f26cb40c0f634528816abea0d66d968b
    """
    my_future = Future()

    def _on_done():
        my_future.set_result(True)
    timer = node.create_timer(timeout_sec, callback=_on_done,
                              callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
    await my_future
    # Important! Clean up the timer properly
    node.destroy_timer(timer)


class ServicesNode(Node):
    """Node which publishes significant data."""

    def __init__(self):
        super().__init__('ServicesNode')
        self.initialize_services()
        self.counter = 0

    def initialize_services(self):
        """qos_profile_services_default=QoSProfile(history=HistoryPolicy.KEEP_L
        AST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE, lifespan=0 nanoseconds,
        deadline=0 nanoseconds, liveliness=LivelinessPolicy.SYSTEM_DEFAULT,
        liveliness_lease_duration=0 nanoseconds,
        avoid_ros_namespace_conventions=False)"""
        reentrant_service_callback_group = ReentrantCallbackGroup()
        self.create_service(RunFakeTask, "service_node/reentrant/run_fake_task",
                            callback=self.run_fake_task_callback,
                            qos_profile=qos_profile_services_default,
                            callback_group=reentrant_service_callback_group)
        self.create_service(RunFakeTask, "service_node/exclusive/run_fake_task",
                            callback=self.run_fake_task_callback,
                            qos_profile=qos_profile_services_default,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info("Created ROS2 services...")

    async def run_fake_task_callback(self, request, response):
        self.counter += 1
        self.get_logger().info(
            f'Received service request: {request.duration_sec}')
        now = perf_counter()
        await sleep(self, request.duration_sec)
        response.success = True
        response.message = f'Processed callback in {perf_counter()-now:.2f} seconds. And here is your lucky number: {self.counter}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServicesNode()
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
