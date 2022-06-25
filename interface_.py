#!/usr/bin/python3
# -*- coding:utf-8 -*-
import time
import threading
from multiprocessing import Queue, Process
from datetime import datetime

main_lock = threading.Lock()
queue: Queue = Queue(1)


def main(user_workspace_dir: str, trace_id: int, res_queue: Queue):
    import rospy
    from timekeeper import JudgeNode, UserTask, SimulatorTask
    result: dict = {"seconds": 0.0,
                    "error": True,
                    "timeout": False,
                    "error_description": ""}

    sim_task = SimulatorTask(trace_id)
    print("wait simulator init for 5 seconds")
    time.sleep(5)
    judge_task = JudgeNode(trace_id)
    # if init simulator error ,return at once
    if sim_task.error_return:
        result["seconds"] = 0.0
        result["error"] = True
        result["error_description"] += sim_task.error_return if sim_task.error_return != None else ""
        judge_task.kill_ros_process()
        res_queue.put(result)
        return

    judge_task.init_task()
    user_task = UserTask(user_workspace_dir, trace_id)
    judge_task.wait_flag = False

    try:
        while not rospy.is_shutdown():
            if judge_task.finish_seconds != None:
                result["error"] = False
                result["seconds"] = judge_task.finish_seconds
                result["timeout"] = judge_task.timeout
                break

            if sim_task.error_return or user_task.error_return:
                result["seconds"] = 0.0
                result["error"] = True
                result["error_description"] += sim_task.error_return if sim_task.error_return != None else ""
                result["error_description"] += user_task.error_return if user_task.error_return != None else ""
                break

            time.sleep(0.1)

    finally:
        judge_task.kill_ros_process()
        res_queue.put(result)
        return


def interface_func(track_id: int, user_workspace_dir: str) -> dict:
    with main_lock:
        main_process: Process = Process(target=main,
                                        args=(user_workspace_dir,
                                              track_id,
                                              queue),
                                        daemon=True)
        main_process.start()
        result = queue.get()
        main_process.join()
    msg = {
        "msg": "timeout" if result["timeout"] == True else
        ("success" if result["error"] == False else str(result["error_description"])),

        "code": -1 if result["error"] == True else (0 if result["timeout"] == False else -1),
        "castTime": int(result["seconds"])
    }
    time.sleep(1)
    print("["+str(datetime.now().strftime('%Y-%m-%d %H:%M:%S'))+"]: ", msg)
    return msg
