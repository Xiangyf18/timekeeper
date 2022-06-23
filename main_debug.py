#!/usr/bin/python3
# -*- coding:utf-8 -*-
import argparse
import sys

from interface_ import *


def parse_args(args, parser: argparse.ArgumentParser):
    parser.add_argument('--track_id', type=int)
    parser.add_argument('--workspace_path', type=str)
    all_args = parser.parse_known_args(args)[0]
    return all_args


def task_main(args):
    parser = argparse.ArgumentParser()
    all_args = parse_args(args, parser)
    return interface_func(track_id=int(all_args.track_id-1),
                          user_workspace_dir=all_args.workspace_path)


if __name__ == '__main__':
    task_main(sys.argv[1:])
