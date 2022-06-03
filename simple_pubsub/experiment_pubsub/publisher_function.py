import rclpy
import sys

from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_sensor_data

from .publisher import NumPublisher, StrPublisher, NumUnboundedArrayPublisher


PUB_TYPE_IDX = 1
QOS_TYPE_IDX = 2


def publisherFactory(pub: str, period: int, qos: int):
    if qos == '0':
        qos = qos_profile_system_default
    elif qos == '1':
        qos = qos_profile_sensor_data

    if pub == 'num':
        return NumPublisher(period, qos)
    elif pub == 'num_array':
        return NumUnboundedArrayPublisher(period, qos)
    elif pub == 'str':
        return StrPublisher(period, qos)


def main(args=None):
    time_period = 0.5
    pub_type = 'num'
    qos_type = 0
    if len(sys.argv) == 3:  # vulnerable here
        pub_type = sys.argv[PUB_TYPE_IDX]
        qos_type= sys.argv[QOS_TYPE_IDX]

    rclpy.init(args=args)
    publisher = publisherFactory(pub=pub_type, period=time_period, qos=qos_type)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
