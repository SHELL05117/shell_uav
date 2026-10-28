# 上游来源与本项目修复

来源：ros-drivers/usb_cam，0.3.7标签，commit `addab4a65fdf65c460fec2cc3eee8fab94699370`。

官方下载地址：https://github.com/ros-drivers/usb_cam/tree/addab4a65fdf65c460fec2cc3eee8fab94699370

保留上游BSD许可证、作者及源码版权声明。2026-10-24按SHELL授权将源码纳入项目工作空间，以覆盖ROS系统包中的同名驱动，系统原包保持可用。

修改集中在资源生命周期：初始化解码器指针；集中且可重复调用shutdown；释放并清空映射缓冲、文件句柄、FFmpeg帧及图像存储，避免节点析构与成员析构重复清理。保留原采集、颜色转换、话题和参数接口，不改变视觉业务。

验收要求包括无设备重复shutdown，以及真实相机MJPEG／YUYV出图后SIGINT正常退出。实际结果记于[当天日志](../../logs/2026-10-24_外设接入、NPU排查与YOLO验收.md)；完成前不视为已经验收。
