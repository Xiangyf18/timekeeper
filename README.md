# Timekeeper

0. 主要依赖库安装
- ros-noetic-desktop 安装: http://wiki.ros.org/noetic/Installation/Ubuntu
- python相关库安装如下：
```
python3 -m pip install rospkg catkin_tools fastapi[all]
```

1. 说明
- 计时基于 `ros time`,与仿真器完全一致,而非真实时间
- 确保仿真赛道启动指令正常，相关配置请见 `config.yaml`

2. 调试方式
```sh
python3 main.py 
```
随后往`http://127.0.0.1:8000/api/timekeeper`发送POST请求
