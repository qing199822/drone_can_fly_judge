# 参与调试

## 提交问题

请附上：

1. Ubuntu、ROS、Gazebo、PX4、MAVROS、GPU 驱动和 Python 版本。
2. 使用的任务 JSON 和完整启动命令。
3. 首次出现错误的时间、无人机编号和相关日志上下文。
4. 六个 MAVROS 的连接状态以及对应图像话题是否收到真实消息。
5. 能稳定复现问题的最短步骤。

日志很大时只截取首次异常前后内容，不要上传整个 `~/.ros/log`。

## 修改要求

- 保持 PX4 `v1.11.0-beta1` 和 Gazebo 11 的兼容性。
- 所有任务航点高度必须低于 6 米。
- 不提交 `build/`、`devel/`、虚拟环境、PX4 固件包、ROS 日志或个人编辑器缓存。
- 修改导航、启动顺序或任务航线时，添加或更新对应回归测试。
- 不覆盖第三方目录原有许可证。

## 最低验证

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 -m unittest discover -s tests -p 'test_*.py'
python3 src/mix_nav/fly/test/test_fly_launch.py
python3 src/mix_nav/task_manager/test/test_mission_clearance.py
rostest simple_navigator velocity_continuity.test
rostest pose_init pose_namespace.test
```

影响 Gazebo、PX4、MAVROS、相机或跟踪链路的改动还需要一次六机运行验证。报告每架是否离地、是否保持连接、是否发送控制指令、RGB/深度图像是否持续发布，以及是否有碰撞或翻覆。

