#!/usr/bin/python3
# -*- coding:utf-8 -*-
from timekeeper import JudgeNode, UserTask, SimulatorTask
import argparse
import sys
import time
import rospy
import os


def parse_args(args, parser: argparse.ArgumentParser):
    parser.add_argument('--id', type=int)
    parser.add_argument('--dir', type=str)
    all_args = parser.parse_known_args(args)[0]
    return all_args


def main(args):
    parser = argparse.ArgumentParser()
    all_args = parse_args(args, parser)

    result: dict = {"seconds": 0.0,
                    "error": False,
                    "error_description": ""}

    sim_task = SimulatorTask(all_args.id)
    print("wait simulator init for 5 seconds")
    time.sleep(5)
    judge_task = JudgeNode(all_args.id)
    # if init simulator error ,return at once
    if sim_task.error_return:
        result["seconds"] = judge_task.max_seconds
        result["error"] = True
        result["error_description"] += sim_task.error_return if sim_task.error_return != None else ""
        judge_task.kill_ros_process()
        print(result)
        return

    judge_task.init_task()
    user_task = UserTask(all_args.dir)

    try:
        while not rospy.is_shutdown():
            if judge_task.finish_seconds != None:
                result["seconds"] = judge_task.finish_seconds
                break

            if sim_task.error_return or user_task.error_return:
                result["seconds"] = judge_task.max_seconds
                result["error"] = True
                result["error_description"] += sim_task.error_return if sim_task.error_return != None else ""
                result["error_description"] += user_task.error_return if user_task.error_return != None else ""
                break

            time.sleep(0.1)

    finally:
        judge_task.kill_ros_process()
        print(result)


if __name__ == '__main__':
    main(sys.argv[1:])
