from rclpy.node import Node

from custom_interfaces.msg import Num, NumUnboundedArray, Str


class BaseSubscriber(Node):
    publisher_name = ''
    topic = ''

    def __init__(self, sub_type, qos_profile) -> None:
        super().__init__(self.publisher_name)
        self.subscription = self.create_subscription(
            sub_type,
            self.topic,
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        self.log(msg)

    def log(self, msg):
        pass


class NumSubscriber(BaseSubscriber):
    publisher_name = 'num_subscriber'
    topic = 'num_topic'

    def __init__(self, qos_profile) -> None:
        super().__init__(Num, qos_profile)

    def log(self, msg):
        self.get_logger().info(f'I heard num: {msg.data}')


class NumUnboundedArraySubscriber(BaseSubscriber):
    publisher_name = 'num_array_subscriber'
    topic = 'num_array_topic'

    def __init__(self, qos_profile) -> None:
        super().__init__(NumUnboundedArray, qos_profile)

    def log(self, msg):
        self.get_logger().info(f'I heard num array: {msg.data[0]}')


class StrSubscriber(BaseSubscriber):
    publisher_name = 'str_subscriber'
    topic = 'str_topic'

    def __init__(self, qos_profile) -> None:
        super().__init__(Str, qos_profile)

    def log(self, msg):
        self.get_logger().info(f'I heard str: {msg.data}')
