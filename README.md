# Timekeeper
- 这是课程的仿真赛道测试的计时程序，计时基于 `ros time`,与仿真器完全一致，而非真实时间。
- 网站后台的计时程序逻辑与本程序一致，上传网页前请先进行本地测试。


## 调试方式
### 本地直接调试
```sh
python3 main_debug.py --track_id 1 --workspace_path /home/robot0/catkin_ws_test.zip
```
#### 输入参数: 
- `track_id` 仿真赛道，1/2/3分别对应初级/中级/高级赛道
- `workspace_path` 用户的ROS工作空间路径，压缩为zip格式
#### 工作空间结构说明: 
所上传的ROS工作空间至少需要包含`nicsrobot_line_follower`功能包，如下所示：
```plaintext
catkin_ws_test
├─src/
│  ├─.../
│  │  ├─nicsrobot_line_follower/
│  │  │  ├─launch/
│  │  │  │  ├─nicsrobot_line_follower.launch
│  │  │  │  ├─nicsrobot_line_follower_2.launch
│  │  │  │  └─nicsrobot_line_follower_3.launch
│  │  │  ├─...(your codes).../
│  │  │  ├─CMakeLists.txt
│  │  │  └─package.xml
│  │  └─.../
│  └─.../
└─.../

```
本程序将自动重新编译整个ROS空间，并根据所选择的赛道，对应执行如下命令，开始计时：
+ `roslaunch nicsrobot_line_follower nicsrobot_line_follower.launch`
+ `roslaunch nicsrobot_line_follower nicsrobot_line_follower_2.launch`
+ `roslaunch nicsrobot_line_follower nicsrobot_line_follower_3.launch`


**用户务必保证程序启动指令与上面格式一致。**


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


## 安装方式
**相关依赖已经在云服务器中安装完毕**，如果另有需要可按以下步骤自行安装依赖库。
- ros-noetic-desktop 安装: http://wiki.ros.org/noetic/Installation/Ubuntu
- python相关库安装如下：
```
python3 -m pip install rospkg catkin_tools fastapi[all]
```