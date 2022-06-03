import os
import sys


PUB_EXTENSION = 'pub.log'
SUB_EXTENSION = 'sub.log'
TOTAL_MSGS = 121


def get_pub_sub_log_filename(path: str):
    return os.path.join(path, PUB_EXTENSION), os.path.join(path, SUB_EXTENSION)


def get_sub_time(pub_id: int, sub_path: str) -> float:
    with open(sub_path, 'r') as sub:
        sub_msgs = sub.read().split('\n')
        for msg in sub_msgs:
            sub_splitted = msg.split(' ')
            if int(sub_splitted[-1]) == pub_id:
                return float(sub_splitted[1][1:-1])
    return 0


def get_total_time_and_success_num(path: str):
    pub_log, sub_log = get_pub_sub_log_filename(path)

    successfully_delivered_msg_num = 0
    total_delivery_time = 0

    with open(pub_log, 'r') as pub:
        pub_msgs = pub.read().split('\n')
        for msg in pub_msgs:
            pub_splited = msg.split(' ')

            pub_id = int(pub_splited[-1])
            pub_time = float(pub_splited[1][1:-1])
            sub_time = get_sub_time(pub_id, sub_log)

            if sub_time == 0:
                continue
            successfully_delivered_msg_num += 1
            total_delivery_time += (sub_time - pub_time)

    return total_delivery_time, successfully_delivered_msg_num

def get_average_msg_delivery_time(path: str) -> float:
    total_delivery_time, successfully_delivered_msg_num = get_total_time_and_success_num(path)

    return total_delivery_time / successfully_delivered_msg_num


def get_msg_loss_rate(path: str) -> float:
    _, successfully_delivered_msg_num = get_total_time_and_success_num(path)

    return (TOTAL_MSGS - successfully_delivered_msg_num) / TOTAL_MSGS


def main():
    directory_path = sys.argv[1]

    average_msg_delivery_time = get_average_msg_delivery_time(directory_path)
    msg_loss_rate = get_msg_loss_rate(directory_path)

    print('average msg delivery time:', average_msg_delivery_time)
    print('msg loss rate:', msg_loss_rate)


if __name__ == '__main__':
    main()
