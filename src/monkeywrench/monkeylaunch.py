#!/usr/bin/env python3

from copy import deepcopy
import sys
from typing import List

import roslaunch
from roslaunch import config
from roslaunch.core import Node

original_load_config = None
config_file = None

def construct_remappings(n, source_nodes: List, source_topics: List):
    remappings = []
    new_node = None
    for t in source_topics:
        if n.name == source_nodes[source_topics.index(t)]:
            index = source_nodes.index(n.name)
            remappings.append(
                [source_topics[index], "{}_mwin".format(source_topics[index])]
            )
            new_node = construct_monkeynode(t, n)
        else:
            remappings.append([t, "{}_mwout".format(t)])
    return n.remap_args + remappings, new_node


def construct_monkeynode(t, source_node: Node):
    n = deepcopy(source_node)
    n.name = n.name + "_mw" if n.name is not None else "_mw"
    n.package = "monkeywrench"
    n.type = "monkeywrench"
    n.remap_args += [["sub", "{}_mwin".format(t)], ["pub", "{}_mwout".format(t)]]
    return n


def new_load_config(
    roslaunch_files,
    port,
    roslaunch_strs=None,
    loader=None,
    verbose=False,
    assign_machines=True,
    ignore_unset_args=False,
):
    assert original_load_config is not None, "Could not import load_config function"
    config = original_load_config(
        roslaunch_files,
        port,
        roslaunch_strs,
        loader,
        verbose,
        assign_machines,
        ignore_unset_args,
    )

    source_topics = []
    source_nodes = []
    assert config_file is not None, "Could not open config file"
    with open(config_file, "r") as t:
        for line in t.readlines():
            stripchars = " \t\r\n\"'"
            node_topic_pair = line.split(":")
            source_nodes.append(node_topic_pair[0].strip(stripchars))
            source_topics.append(node_topic_pair[1].strip(stripchars))

    new_nodes = []
    for n in config.nodes:
        n.remap_args, new_node = construct_remappings(n, source_nodes, source_topics)
        if new_node is not None:
            new_nodes.append(new_node)
    config.nodes += new_nodes
    return config

def main():
    global original_load_config
    global config_file
    original_load_config = config.load_config_default
    config_file = sys.argv[-1]
    config.load_config_default = new_load_config
    roslaunch.main(sys.argv[:-1])
