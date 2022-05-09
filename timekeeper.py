#!/usr/bin/python3
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import os
import subprocess
import threading


class JudgeNode:
    def __init__(self,max_time):
        self.max_time=max_time

        # for ros init 
        rospy.init_node('judge_node_', anonymous=True)
        self.rate = rospy.Rate(10)
        self.JudgeInfoSub = rospy.Subscriber('/AKM_1/odom', Odometry, self.JudgeCallback)

        # for position init
        self.pose=[0.0,0.0]
        self.half_check=False 
    
    def main_task(self)->None:
        self.start_time=rospy.get_time()
        while not rospy.is_shutdown():
            self.time_now=rospy.get_time()
            x=self.pose[0]
            y=self.pose[1]
            
            # 确认跑了半圈 TODO
            if abs(x-0.625)<=0.05 and abs(y+1.75)<1.2:
                self.half_check=True

            # 确认到达初始点
            if self.half_check==True and abs(x-0.625)<=0.05 and abs(y+1.75)<1.2:
                self.finish_time=rospy.get_time()
                return float(self.finish_time-self.start_time)
            
            # 超过最长用时
            if (self.time_now-self.start_time)>self.max_time:
                return float(self.max_time)
            
            self.rate.sleep()
            


    def JudgeCallback(self,msg:Odometry):
        self.pose[0]=msg.pose.pose.position.x
        self.pose[1]=msg.pose.pose.position.y


class ROSTask:
    def __init__(self, workspace_dir_path:str):
        self.dir_path = workspace_dir_path

        self.killed = False
        self.error_return = None # 遇到错误，意外退出

        #重新编译文件夹
        cmd="cd "+self.dir_path+" && rm -rf build/ devel/ && catkin_make"
        os.popen(cmd)  

        #执行
        exec_cmd = "source "+self.dir_path+"/devel/setup.bash && roslaunch nicsrobot_line_follower nicsrobot_line_follower.launch"
        self.ros_driver_process = subprocess.Popen(["/bin/bash", "-c", exec_cmd],
                                                   shell=False, stdin=subprocess.PIPE,
                                                   stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                                   encoding="utf-8", preexec_fn=os.setsid)  # 创建进程组
        # 检测意外错误
        self.wait_exit_thread = threading.Thread(target=self.exit_info, daemon=True)
        self.wait_exit_thread.start()

    def exit_info(self):
        return_data = self.ros_driver_process.communicate(input=None)
        if self.killed == False:
            self.error_return=return_data
            print(
                f'[DEBUG]: ROSDriver process {self.ros_domain_id} unexpectedly exit with data:: {return_data}')

    def kill_process(self):
        try:
            if self.killed == False:
                os.killpg(os.getpgid(self.ros_driver_process.pid), 9)
                self.ros_driver_process.wait()
                self.killed = True
                pass
        except Exception as e:
            print(f'[ERROR]: raise exception \'{e}\' in function \'kill_process\'')

    def __del__(self):
        self.kill_process()


class GazeboTask:
    TODO:
    pass