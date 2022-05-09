# Timekeeper


1. 说明
- 计时基于ros time,与仿真器完全一致,而非真实时间
- 假设了用户上传了整个catkin_ws文件（可以不编译）
- 假设了用户的跑圈程序接口为
    - "roslaunch  nicsrobot_line_follower nicsrobot_line_follower.launch"

2. 调试方式
```sh
# 输入用户文件夹路径，以及比赛最长用时
python3 main.py /home/unbuntu/catkin_ws 300
# 结果:dict type
# {"result":160.14292,"error":False,"error_description":""}
```

3. TODO
协商与网页后端的通信接口形式，http?还是os直接调用？