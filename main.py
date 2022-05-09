#!/usr/bin/python3
# -*- coding:utf-8 -*-


TODO:
# timekeeper脚本中
# 增加gazebo的子进程调用
# judgeNode 以小车动了为起点开始
# judgeNode 确认跑了半圈在哪里pose

# 本脚本
# 子进程打开gazebo，注意接受回调错误
# 子线程打开timekeeper
# 子进程打开rostask，注意接受回调错误
# 持续等待，timekeeper 结果更新了/另外两个出现意外结束错误（非None）
# 返回结果：dict


# 佳昊的仓库
# 重新写计时函数

# 验证：
# localhost设置会不会影响用户使用（bashrc怎么写的我有些忘了）
# 无错误时，计时符合预计
# 有错误时，错误输出符合预计

#TODO:
# 高级障碍赛的初始位置不提供
# 需要输入比赛id，最长用时就由我默认了