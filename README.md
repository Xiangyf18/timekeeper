# Timekeeper


1. 说明
- 计时基于 `ros time`,与仿真器完全一致,而非真实时间
- 相关配置请见 `config.yaml`

2. 调试方式
```sh
# 输入赛道序号和用户文件夹路径
python3 main.py --id 0  --dir /home/unbuntu/catkin_ws
# 结果:dict type
# {"result":160.14292,"error":False,"error_description":""}
```

3. TODO
- 协商与网页后端的通信接口形式，http?还是os直接调用？
- 比赛的其他几个跑道，确定一下pose路标
- 开课前把各种意外情况都测试一下