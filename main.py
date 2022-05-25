#!/usr/bin/python3
# -*- coding:utf-8 -*-
from timekeeper import JudgeNode, UserTask, SimulatorTask
import time
import rospy
import uvicorn
from fastapi import FastAPI
from fastapi import Request


def main(user_workspace_dir: str, trace_id: int = 0):

    result: dict = {"seconds": 0.0,
                    "error": False,
                    "error_description": ""}

    sim_task = SimulatorTask(trace_id)
    print("wait simulator init for 5 seconds")
    time.sleep(5)
    judge_task = JudgeNode(trace_id)
    # if init simulator error ,return at once
    if sim_task.error_return:
        result["seconds"] = -1.0
        result["error"] = True
        result["error_description"] += sim_task.error_return if sim_task.error_return != None else ""
        judge_task.kill_ros_process()
        print(result)
        return result

    judge_task.init_task()
    user_task = UserTask(user_workspace_dir)

    try:
        while not rospy.is_shutdown():
            if judge_task.finish_seconds != None:
                result["seconds"] = judge_task.finish_seconds
                break

            if sim_task.error_return or user_task.error_return:
                result["seconds"] = -1.0
                result["error"] = True
                result["error_description"] += sim_task.error_return if sim_task.error_return != None else ""
                result["error_description"] += user_task.error_return if user_task.error_return != None else ""
                break

            time.sleep(0.1)

    finally:
        judge_task.kill_ros_process()
        print(result)
        return result


app = FastAPI()


@app.post("/api/timekeeper")
async def home(request: Request):
    data = await request.json()
    result = main(user_workspace_dir=data["actualPath"], trace_id=0)
    msg = {
        "msg": "success" if result["error"] == False else "fail",
        "code": 0 if result["error"] == False else -1,
        "castTime": int(result["seconds"])
    }
    return msg


if __name__ == '__main__':
    uvicorn.run(app, host='127.0.0.1', port=8000, debug=True)
