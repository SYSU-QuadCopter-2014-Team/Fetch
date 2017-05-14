# Fetch

无人机自主抓取控制模块，原始修改自TA的KCF_tracking代码。

基于TA代码中的Onboard-SDK-3.0源码库，也依赖识别组的图像坐标识别，和标定组的坐标换算。

原本的DJI_tracking现作为UI，但保留了如摄像头上下左右的控制代码，而抓取模块将移到Fetch类中实现。

目前主功能模块Fetch有2个功能函数：

1. moveByPositionOffset：执行位置控制任务。给定坐标偏移量进行PID控制飞行。用于测试PID控制效果。
2. approaching：根据runKCF线程实时计算的位置信息（坐标偏移量）进行PID控制，使飞机飞到目标物体上方后再下降。同时将加入摄像头的控制，以跟随目标移动。
