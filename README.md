# Timekeeper

## 主要依赖库安装
相关依赖已经在云服务器中安装完毕。
- ros-noetic-desktop 安装: http://wiki.ros.org/noetic/Installation/Ubuntu
- python相关库安装如下：
```
python3 -m pip install rospkg catkin_tools fastapi[all]
```

## 说明
- 本地计时程序与网页后端计时程序相同，上传网页前请先本地测试。
- 计时基于 `ros time`,与仿真器完全一致，而非真实时间。
- 确保程序启动指令格式正确，三个赛道对应的指令为：
    + `roslaunch nicsrobot_line_follower nicsrobot_line_follower.launch"`
    + `roslaunch nicsrobot_line_follower nicsrobot_line_follower_2.launch"`
    + `roslaunch nicsrobot_line_follower nicsrobot_line_follower_3.launch"`


## 调试方式
### 本地直接调试
```sh
python3 main_debug.py --track_id 1 --workspace_path /home/robot0/catkin_ws_test.zip
```
#### 输入参数: 
- `track_id` 仿真赛道，1/2/3分别对应初级/中级/高级赛道
- `workspace_path` 用户的工作空间路径，压缩为zip格式
#### 输出结果：
- `code` 状态码，结果为0代表成功，非0代表错误
- `msg` 状态描述，结果为 `success`或者`timeout`或者具体报错信息
- `castTime` 花费时间，单位：秒



### http请求接口
```sh
python3 main_web.py 
```
#### 输入
- 输入方式：往`http://127.0.0.1:8000/api/timekeeper`发送POST请求
- 工具推荐：可使用vscode的REST Client 插件直接发送http请求
- 数据格式：具体参考`test_http/test.http`文件的数据格式
#### 输出结果
与本地直接调试结果一致

