#!/usr/bin/env python
import rosnode


def get_move_group_feedback_topics():
    info = rosnode.get_node_info_description("/move_group").split("\n")
    filtered = filter(check, info)
    return filtered

def check(topic):
    if "feedback" in topic:
        return True
    else:
        return False
