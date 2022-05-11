#!/usr/bin/env python3

import rospy
import numpy as np
from rospy.msg import AnyMsg
from roslib.message import get_message_class
from typing import List
from monkeywrench.srv import TriggerError, TriggerErrorRequest, TriggerErrorResponse


class MonkeyWrench:
    def __init__(self, node_name, input_topic, output_topic):
        self.msg_type = None

        def _raw_handler(msg: AnyMsg):
            self.msg_type = msg._connection_header["type"]

        raw_sub = rospy.Subscriber(input_topic, AnyMsg, _raw_handler)
        while self.msg_type is None and not rospy.is_shutdown():
            pass
        raw_sub.unregister()
        itype = get_message_class(self.msg_type)

        self.sub = rospy.Subscriber(input_topic, itype, self.handle_sub, queue_size=10)
        self.pub = rospy.Publisher(output_topic, itype, queue_size=10)
        self.trigger = rospy.Service(
            "trigger_error_{}".format(node_name), TriggerError, self.trigger_error_cb
        )
        self.end_time = rospy.Time(0)
        self.has_modifications = False
        self.should_timeout = False
        self.stop_sending = False
        self.use_constant = False
        self.send_gaussian = False
        self.send_uniform = False
        self.changetime = False

        self.constant = 0
        self.noise_std_dev = 0
        self.time_change = 0
        self.apply_to_subtopics: List[str] = []

    def handle_sub(self, msg):
        if self.should_timeout and rospy.Time().now() > self.end_time:
            # past our timeout
            self.has_modifications = False

        if not self.has_modifications:
            # no modifications, publish verbatim
            self.pub.publish(msg)
            return

        # if we're here it's because we need to tweak a message.
        # first, do we plan on tweaking the time?
        if self.changetime:
            msg = self.tweak_time(msg)

        func = self.generate_function()
        self.walk_and_apply("", msg, func, self.subtopics)

    def generate_function(self):
        def generated(value):
            data_center = value
            if self.use_constant:
                data_center = self.constant

            if self.send_gaussian:
                return np.random.normal(data_center, self.noise_std_dev)  # type:ignore

            if self.send_uniform:
                return np.random.uniform(  # type:ignore
                    data_center + self.noise_std_dev, data_center - self.noise_std_dev
                )

        return generated

    def walk_and_apply(self, message_subtopic, msg, func, filter_in):
        for slot in msg.__slots__:
            if not self.allowed_subtopic(message_subtopic, slot, filter_in):
                return  # skip
            attr = msg.__getattribute__(slot)
            if type(attr) is int or type(attr) is float:
                msg.__setattribute__(slot, func(msg.__getattribute__(slot)))
            else:
                new_message_subtopic = message_subtopic + "/" + str(attr)
                self.walk_and_apply(
                    new_message_subtopic, msg.__getattribute__(slot), func, filter_in
                )

    def allowed_subtopic(self, message_subtopic, attr, filter_in):
        combined = ("{}/{}".format(message_subtopic, attr)).lstrip('/')
        if len(filter_in) == 0:
            return True
        return combined in filter_in

    def tweak_time(self, msg):
        # planning to change the time implies that there's a header we can play with.
        types = msg._get_types()
        assert type(types) is List[str]

        if "std_msgs/Header" not in types:
            rospy.logwarn("attempted to adjust time on message which has no header")
            return msg

        if "header" in msg.__slots__:
            msg.header += rospy.Duration(self.time_change)
            return msg

        # this gets the cases where the header isn't called 'header' by
        # introspecting on the message for the std_msgs/Header type.
        msg.__getattribute__(
            msg.__slots__[msg._slot_types.index("std_msgs/Header")]
        ).stamp += rospy.Duration(self.time_change)

        return msg

    def apply_operation(self, current_value):
        return current_value

    def trigger_error_cb(self, srv: TriggerErrorRequest):
        # Collect set of failure cases. Using this bitwise-and approach to
        # enable multiple simultaneous failure cases on one topic.
        # fmt:off
        # (autoformatting would obscure what's happening here so let's just ignore it)
        stop_sending  = True if (srv.failure_mode & srv.FAIL_STOP_SENDING)        == srv.FAIL_STOP_SENDING        else False
        use_constant  = True if (srv.failure_mode & srv.FAIL_USE_CONSTANT)        == srv.FAIL_USE_CONSTANT        else False
        send_gaussian = True if (srv.failure_mode & srv.FAIL_SEND_GAUSSIAN_NOISE) == srv.FAIL_SEND_GAUSSIAN_NOISE else False
        send_uniform  = True if (srv.failure_mode & srv.FAIL_SEND_UNIFORM_NOISE)  == srv.FAIL_SEND_UNIFORM_NOISE  else False
        changetime    = True if (srv.failure_mode & srv.FAIL_CHANGETIME)          == srv.FAIL_CHANGETIME          else False
        # fmt:on

        self.has_modifications = (
            stop_sending or use_constant or send_gaussian or send_uniform or changetime
        )
        if not self.has_modifications:
            return

        self.should_timeout = False
        if srv.duration > 0:
            # user has specified non-zero timeout
            self.end_time = rospy.Time().now() + rospy.Duration(srv.duration)

        # sanity checks. Some of these errors can coexist, but it doesn't make
        # sense to have both gaussian and uniform noise at the same time (for
        # example).

        if stop_sending and not srv.failure_mode == srv.FAIL_STOP_SENDING:
            # user has specified topic should stop sending, but has also
            # specified that it should send something else. That doesn't really
            # make sense, so let's warn them about that.
            rospy.logwarn(
                "Invalid error specification: sent 'stop sending' command with \
                other commands. Ignoring 'stop sending' command."
            )
            stop_sending = False
        if send_gaussian and send_uniform:
            rospy.logwarn(
                "Invalid error specification: sent goal of both gaussian and \
                uniform noise. Defaulting to gaussian."
            )
            send_uniform = False

        self.stop_sending = stop_sending
        self.use_constant = use_constant
        self.send_gaussian = send_gaussian
        self.send_uniform = send_uniform
        self.changetime = changetime

        self.constant = srv.constant
        self.noise_std_dev = srv.noise_std_dev if srv.noise_std_dev is not 0 else 1
        self.time_change = srv.time_change

        self.subtopics = srv.apply_to_subtopics

        return TriggerErrorResponse(True)

    def clean_shutdown(self):
        self.sub.unregister()
        self.pub.unregister()


def main():
    input_topic = rospy.get_param("input", "sub")
    output_topic = rospy.get_param("output", "pub")

    input_tail = input_topic.split("/")[-1]
    output_tail = output_topic.split("/")[-1]
    node_name = "monkeywrench_{}_{}".format(input_tail, output_tail)
    rospy.init_node(node_name, anonymous=True)
    mw = MonkeyWrench(node_name, input_topic, output_topic)
    rospy.on_shutdown(mw.clean_shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
