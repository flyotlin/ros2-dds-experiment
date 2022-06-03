import rclpy
import sys

from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_sensor_data

from .subscriber import NumSubscriber, StrSubscriber, NumUnboundedArraySubscriber


SUB_TYPE_IDX = 1
QOS_TYPE_IDX = 2


def subscriberFactory(sub: str, qos: int):
    if qos == '0':
        qos = qos_profile_system_default
    elif qos == '1':
        qos = qos_profile_sensor_data

    if sub == 'num':
        return NumSubscriber(qos)
    elif sub == 'num_array':
        return NumUnboundedArraySubscriber(qos)
    elif sub == 'str':
        return StrSubscriber(qos)

def main(args=None):
    sub_type = 'num'
    qos_type = 0
    if len(sys.argv) == 3:  # vulnerable here
        sub_type = sys.argv[SUB_TYPE_IDX]
        qos_type= sys.argv[QOS_TYPE_IDX]

    rclpy.init()
    subscriber = subscriberFactory(sub_type, qos_type)
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
