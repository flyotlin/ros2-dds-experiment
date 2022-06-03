from rclpy.node import Node

from custom_interfaces.msg import Num, NumUnboundedArray, Str


class BasePublisher(Node):
    publisher_name = ''
    topic = ''

    def __init__(self, time_period: float, pub_type, qos_profile) -> None:
        super().__init__(self.publisher_name)
        self.publisher = self.create_publisher(pub_type, self.topic, qos_profile)
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = self.get_msg()
        self.publisher.publish(msg)
        self.log(msg)
        self.i += 1

    def get_msg(self):
        pass

    def log(self, msg):
        pass


class NumPublisher(BasePublisher):
    publisher_name = 'num_publisher'
    topic = 'num_topic'

    def __init__(self, time_period: float, qos_profile) -> None:
        super().__init__(time_period, Num, qos_profile)

    def get_msg(self):
        msg = Num()
        msg.data = self.i
        return msg

    def log(self, msg):
        self.get_logger().info(f'Publishing num: {msg.data}')


class NumUnboundedArrayPublisher(BasePublisher):
    publisher_name = 'num_array_publisher'
    topic = 'num_array_topic'

    def __init__(self, time_period: float, qos_profile) -> None:
        super().__init__(time_period, NumUnboundedArray, qos_profile)

    def get_msg(self):
        msg = NumUnboundedArray()
        msg.data = [x for x in range(self.i, self.i + 1000000)]
        return msg

    def log(self, msg):
        self.get_logger().info(f'Publishing num array: {msg.data[0]}')


class StrPublisher(BasePublisher):
    publisher_name = 'str_publisher'
    topic = 'str_topic'

    def __init__(self, time_period: float, qos_profile) -> None:
        super().__init__(time_period, Str, qos_profile)

    def get_msg(self):
        msg = Str()
        msg.data = f'strstrstr-{self.i}'
        return msg

    def log(self, msg):
        self.get_logger().info(f'Publishing str: {self.i}')
