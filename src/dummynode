#!/usr/bin/env python3

import rospy
from roslib.message import get_message_class

def main():
    topic_type = rospy.get_param('topic_type',      'std_msgs/Float32')
    topic_pub  = rospy.get_param('publish_topic',   '')
    hz_pub     = rospy.get_param('publish_hz',      15)
    topic_sub  = rospy.get_param('subscribe_topic', '')

    rospy.init_node('dummynode', anonymous=True)

    def cb(msg):
        del msg

    message_type = get_message_class(topic_type)
    assert message_type is not None
    if len(topic_sub) != 0:
        rospy.Subscriber(topic_sub, message_type, cb)
    publisher = None
    if len(topic_sub) != 0:
        publisher = rospy.Publisher(topic_pub, message_type, queue_size=5)

    rate = rospy.Rate(hz_pub)
    while not rospy.is_shutdown():
        if publisher is not None:
            publisher.publish(message_type())
        rate.sleep()

if __name__ == '__main__':
    main()
