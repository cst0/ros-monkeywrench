import rospy
import argparse
import rosgraph
from rospy.msg import AnyMsg


class TopicHistory:
    def __init__(self, topic, window=50):
        self.topic = topic
        self.window = window
        self.history = []
        self.subscriber = rospy.Subscriber(topic, AnyMsg, self.callback, queue_size=1)

    def append(self, msg):
        self.history.append(msg)
        if len(self.history) > self.window:
            self.history.pop(0)

    def callback(self, msg):
        self.append(msg)

    def shutdown(self):
        self.subscriber.unregister()


class NodeFeatureAnalyzer:
    def __init__(self, node):
        master = rosgraph.Master("NodeFeatureAnalyzer")

        def find_topic_type(t, topic_types):
            matches = [t_type for t_name, t_type in topic_types if t_name == t]
            if matches:
                return matches[0]
            return "unknown type"

        state = master.getSystemState()
        topic_types = master.getTopicTypes()

        pubs, _, _ = state  # type:ignore
        pubs_out = []
        for topic, nodes in pubs:
            pubs_out.append((topic, find_topic_type(topic, topic_types), nodes))

        self.history = {}
        for p in pubs_out:
            self.history[p] = TopicHistory(p)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("node", required=True, help="target node")
    args = parser.parse_args(rospy.myargv())
    NodeFeatureAnalyzer(args.node)
