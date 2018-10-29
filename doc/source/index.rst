.. autolabor_simulation documentation master file, created by
   sphinx-quickstart on Mon May 28 23:42:15 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

======================================
Autolabor_simulation Documentation
======================================

Autolabor Simulation 是什么
==============================

Autolabor Simulation是一款基于ROS（Robot Operating System）的轻量级开源轮式机器人模拟器。其能实现轮速里程计，激光雷达，机器人底盘模拟，并可自定义二维场景以及动态障碍物。并提供丰富参数配置以及接口对相应传感器以及场景的操控。

Autolabor Simulation使用ROS开发，不依赖与其他软件工具，适用于初学者学习ROS以及开发者调试导航，建图等算法。其最大的特点就是简单易上手，仅通过配置相应ROS节点即可模拟获取多种传感器数据，替换相应设备驱动即可快速移植到真实移动机器人上。


Autolabor Simulation 包含哪些模块
===============================
* autolabor_description       —— 机器人urdf模型
* autolabor_simulation_base   —— 轮式机器人底盘模拟
* autolabor_simulation_lidar  —— 单线激光雷达模拟
* autolabor_simulation_stage  —— 基于灰度图片的自定义场景模拟
* autolabor_simulation_object —— 自定义形状的可移动障碍我模拟
* autolabor_keyboard_control  —— 键盘控制模块

Autolabor Simulation 能实现哪些功能
===================================
Autolabor Simulation 在设计时采用低耦合设计观念构建所有模块，有非常良好的可扩展性，后期根据需求会逐渐添加更多传感器（惯导，超声波传感器，毫米波）模拟。现有的功能可实现但不限于以下功能：

* 键盘控制小车运动功能
* Cartographer2D 单线激光雷达SLAM建图
* Gmapping 单线激光雷达SLAM建图
* Amcl 基于单线激光雷达定位
* Navigation 自动导航功能

模拟器提供动态障碍物模拟，对于有一定开发经验的用户可以提供利用其进行激光雷达进行深度强化学习，训练局部规划算法。

教程目录
==========

.. toctree::
   :maxdepth: 2

   Autolabor Simulation 使用入门 <getting_start>
   Autolabor Simulation 预备知识——坐标轴与坐标系 <tf_coordinate_introduction>
   Autolabor Simulation Base 模块介绍 <autolabor_simulation_base>
