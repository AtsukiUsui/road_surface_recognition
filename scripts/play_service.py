#!/usr/bin/env python

import rospy
import std_msgs.msg
import subprocess
import time
import sensor_msgs.msg

# コールバック関数：/diag_scan トピックからメッセージを受信したときに呼ばれる


def diag_scan_callback(msg):
    # メッセージを受信したら、タイムスタンプを更新
    global last_msg_time
    last_msg_time = rospy.get_time()

    # メッセージを受け取ったことをROSOUTでログに出力
    rospy.loginfo("Received a message from /disting_cloud2!")


if __name__ == '__main__':
    rospy.init_node('diag_scan_monitor')

    # サブスクライバを設定し、/diag_scan トピックのメッセージを監視
    rospy.Subscriber('/disting_cloud2',
                     sensor_msgs.msg.PointCloud2, diag_scan_callback)

    # 最後のメッセージ受信時刻を初期化
    last_msg_time = rospy.get_time()

    # rosservice call を実行したかどうかを示すフラグ
    called_service = False

    # メインループ
    rate = rospy.Rate(1)  # 1Hzのループ
    while not rospy.is_shutdown():
        # 最後のメッセージ受信から10秒以上経過していれば、rosservice call を実行
        current_time = rospy.get_time()
        if current_time - last_msg_time > 10.0 and not called_service:
            rospy.logwarn(
                "No messages received for 10 seconds. Calling rosservice...")
            try:
                # rosservice call コマンドを実行
                subprocess.call(["rosservice", "call", "/up_map"])
                called_service = True  # サービスを一度だけ呼び出す
            except Exception as e:
                rospy.logerr("Error calling rosservice: {}".format(e))

        if called_service:
            rospy.loginfo("rosservice call completed. Exiting...")
            break

        rate.sleep()
