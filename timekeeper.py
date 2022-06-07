#!/usr/bin/python3
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import os
import subprocess
import threading
import yaml
import math
import time

# for config
dir_path = os.path.dirname(os.path.realpath(__file__))
config_url = dir_path+"/config.yaml"
with open(str(config_url), mode='r', encoding='utf-8') as f:
    config_data: dict = yaml.load(f, Loader=yaml.FullLoader)


class JudgeNode:
    def __init__(self, race_id):
        # static data
        self.max_seconds = config_data["max_seconds"]
        self.init_pose = config_data["racetrack"][race_id]["init_pose"]
        self.half_pose = config_data["racetrack"][race_id]["half_pose"]
        self.track_width = config_data["racetrack"][race_id]["track_width"]

    def init_task(self):
        # for ros init
        rospy.init_node('judge_node_', anonymous=True)
        self.rate = rospy.Rate(10)
        self.JudgeInfoSub = rospy.Subscriber('/AKM_1/odom', Odometry, self.JudgeCallback)

        # for position init
        self.pose = None
        self.pose_first = None
        self.half_check = False
        self.start_check = False
        self.finish_seconds = None

        self._main_thread = threading.Thread(target=self._main_task, daemon=True)
        self._main_thread.start()

    def _main_task(self) -> None:
        self.start_time = rospy.get_time()
        while not rospy.is_shutdown():
            self.time_now = rospy.get_time()

            if self.start_check and abs(self.pose[0]-self.half_pose["x"]) <= 0.2 \
                    and abs(self.pose[1]-self.half_pose["y"]) < self.track_width:
                self.half_check = True

            if self.half_check == True and abs(self.pose[0]-self.init_pose["x"]) <= 0.2 \
                    and abs(self.pose[1]-self.init_pose["y"]) < self.track_width:
                self.finish_time = rospy.get_time()
                self.finish_seconds = float(self.finish_time-self.start_time)
                return

            if (self.time_now-self.start_time) > self.max_seconds:
                self.finish_seconds = float(self.max_seconds)
                return
            self.rate.sleep()

    def JudgeCallback(self, msg: Odometry):
        new_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.pose = new_pose
        if self.pose_first == None:
            self.pose_first = new_pose
        if math.sqrt((self.pose[0]-self.pose_first[0])**2+(self.pose[1]-self.pose_first[0])**2) > 0.2:
            self.start_check = True

    def kill_ros_process(self):
        os.system(dir_path+"/kill.sh")


class UserTask:
    def __init__(self, workspace_dir_path: str):
        self.workspace_dir_path: str = workspace_dir_path
        self.user_cmd: str = config_data["user_cmd"]

        self.killed = False
        self.error_return = None

        # unzip the workspace
        if config_data["unzip_workspace"] == True:
            post_name = self.workspace_dir_path.split(".")[-1]
            if post_name != "zip":
                self.error_return = "UserTask process unexpectedly exit with data:: File should be a zip type!"
                return
            cmd = "rm rf ~/user_ws && unzip -d ~/user_ws "+self.workspace_dir_path
            os.popen(cmd)

            # get real user workspace
            dirs = os.listdir("~/user_ws")
            if len(dirs) != 1:
                self.error_return = "UserTask process unexpectedly exit with data:: After unzip, workspace format is wrong!"
                return
            self.workspace_dir_path = "~/user_ws/"+str(dirs[0])

        # rebuild workspace
        if config_data["rebuild_workspace"] == True:
            cmd = "cd "+self.workspace_dir_path+" && rm -rf build/ devel/ && catkin_make"
            os.popen(cmd)

        # exec user codes
        exec_cmd = f"source {self.workspace_dir_path}/devel/setup.bash && {self.user_cmd}"
        self.ros_driver_process = subprocess.Popen(["/bin/bash", "-c", exec_cmd],
                                                   shell=False, stdin=subprocess.PIPE,
                                                   stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                                   encoding="utf-8", preexec_fn=os.setsid)  # 创建进程组

        self.wait_exit_thread = threading.Thread(target=self._exit_info, daemon=True)
        self.wait_exit_thread.start()

    def _exit_info(self):
        out, err = self.ros_driver_process.communicate(input=None)
        if self.killed == False:
            self.error_return = '  UserTask process unexpectedly exit with data:: '
            self.error_return += (err if err is not None else "")
            self.killed = True
            # print(f'[ERROR]: {self.error_return}')

    def kill_process(self):
        try:
            if self.killed == False:
                self.killed = True
                os.killpg(os.getpgid(self.ros_driver_process.pid), 9)
                self.ros_driver_process.wait()
                pass
        except Exception as e:
            print(f'[WARNING]: raise exception \'{e}\' in UserTask\'s function:  \'kill_process\'')

    def __del__(self):
        self.kill_process()


class SimulatorTask:
    def __init__(self, race_id) -> None:
        self.killed = False
        self.error_return = None

        try:
            self.cmd = config_data["racetrack"][race_id]["cmd"]
            self.workspace_dir_path = config_data["racetrack"]["workspace_path"]
        except:
            self.error_return = f"SimulatorTask process unexpectedly exit with data:: Can't find config for this race track {race_id} !"
            return

        # exec  simulator
        exec_cmd = f"source {self.workspace_dir_path}/devel/setup.bash && {self.cmd}"
        self.ros_driver_process = subprocess.Popen(["/bin/bash", "-c", exec_cmd],
                                                   shell=False, stdin=subprocess.PIPE,
                                                   stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                                   encoding="utf-8", preexec_fn=os.setsid)  # 创建进程组

        self.wait_exit_thread = threading.Thread(target=self._exit_info, daemon=True)
        self.wait_exit_thread.start()

    def _exit_info(self):
        out, err = self.ros_driver_process.communicate(input=None)
        if self.killed == False:
            self.error_return = '  SimulatorTask process unexpectedly exit with data:: '
            self.error_return += (err if err is not None else "")
            self.killed = True
            # print(f'[ERROR]: {self.error_return}')

    def kill_process(self):
        try:
            if self.killed == False:
                self.killed = True
                os.killpg(os.getpgid(self.ros_driver_process.pid), 9)
                self.ros_driver_process.wait()
                pass
        except Exception as e:
            print(
                f'[WARNING]: raise exception \'{e}\' in SimulatorTask\'s function:  \'kill_process\'')

    def __del__(self):
        self.kill_process()
