使用指导
========

依赖
----

[Autolabor\_simulation](https://github.com/gsc07/autolabor_simulation) 是基于ROS开发的轮式机器人模拟器，所以在使用前需要先安装ROS环境。如果你使用的是Ubuntu 16.04操作系统，ROS环境安装方式可参考 [ROS环境安装](http://wiki.ros.org/kinetic/Installation/Ubuntu)

如果你没有Ubuntu系统环境，可以在模拟器中安装 AutolaborOS，并在 AutolaborOS 中实现教程中所有功能。AutolaborOS 是基于 Ubuntu16.04 定制的机器人操作系统，其中包含ROS环境以及多种传感器驱动，同时包含模拟器以及导航建图功能模块。具体安装方式以及下载地址可以参考 [AutolaborOS安装](http://www.autolabor.com.cn/lib/video/play/4)

编译安装
--------

[Autolabor\_simulation](https://github.com/gsc07/autolabor_simulation) 仓库由若干个 [Ros Package](http://wiki.ros.org/Packages) 组成，仓库下的每个package 有自己的配置文件（package.xml），可以直接将 [Autolabor\_simulation](https://github.com/gsc07/autolabor_simulation) 克隆到你 的workspace的src文件夹下，利用catkin\_make编译autolabor\_simulation.

创建你的 [catkin\_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

``` {.sourceCode .bash}
# Create catkin_workspace
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

下载并编译

``` {.sourceCode .bash}
# Clone Autolabor_simulation to your workspace
cd ~/catkin_ws/src/
git clone https://github.com/gsc07/autolabor_simulation.git

# Build source
cd ..
catkin_make
source devel/setup.bash
```

如果希望ROS环境中默认包含autolabor\_simulation，可以使用如下命令将workspace设为默认包含

``` {.sourceCode .bash}
# Source workspace for every terminal
source ~/catkin_ws/devel/setup.bash
```

Quick Start
-----------

在[simulation\_base](https://github.com/gsc07/autolabor_simulation/tree/master/simulation_launch/launch)中提供了很多预先配好的launch文件，下面运行一个简单的入门教程，使用autolabor\_simulation运行gmapping和move\_base控制小车在迷宫中行走:

``` {.sourceCode .bash}
# Source workspace for every terminal
source ~/catkin_ws/devel/setup.bash
roslaunch simulation_launch gmapping_navigation.launch
```

运行命令后，rviz会自动弹出并显示对应的信息，下面利用rviz中的 2D Nav Goal 来触发小车的自动规划吧！

更多教程，可参考[doc](https://github.com/gsc07/autolabor_simulation/tree/master/doc)文件夹下的使用说明。生成命令如下：

``` {.sourceCode .bash}
# Install doc generator and its dependence
sudo apt-get install python3-sphinx
pip3 install --user sphinx_rtd_theme
# Go to the doc dir and build it
cd ~/catkin_make/src/autolabor_simulation/doc
make html
# Waiting for the make till it finished
xdg-open build/html/index.html
```

项目结构
--------

``` {.sourceCode .bash}
├── autolabor_description             移动机器人urdf模型
│   ├── CMakeLists.txt
│   ├── launch                        启动移动机器人模型相关launch脚本
│   ├── meshes                        移动机器人三维模型
│   ├── package.xml
│   ├── rviz                          移动机器人rviz可视化配置参数
│   └── urdf                          移动机器人三维模型相关配置参数
├── autolabor_keyboard_control        键盘控制模块
│   ├── CMakeLists.txt
│   ├── include                       键盘控制模块头文件
│   ├── package.xml
│   └── src                           键盘控制模块源代码
├── autolabor_simulation_base         移动机器人底盘模拟(包含轮速里程计模拟)
│   ├── CMakeLists.txt
│   ├── include                       底盘模块头文件
│   ├── launch                        底盘模块测试launch脚本
│   ├── package.xml
│   └── src                           底盘模块源代码
├── autolabor_simulation_lidar        单线激光雷达模拟
│   ├── CMakeLists.txt
│   ├── include                       单线激光雷达模块头文件
│   ├── launch                        单线激光雷达测试launch脚本
│   ├── package.xml
│   └── src                           单线激光雷达源代码
├── autolabor_simulation_object       动态障碍物模拟
│   ├── CMakeLists.txt
│   ├── include                       动态障碍物模拟模块头文件
│   ├── launch                        动态障碍物模拟模块launch脚本
│   ├── package.xml
│   └── src                           动态障碍物模拟模块源代码
├── autolabor_simulation_stage        自定义场景模拟
│   ├── CMakeLists.txt
│   ├── include                       自定义场景模拟模块头文件
│   ├── launch                        自定义场景模拟模块测试launch脚本
│   ├── map                           场景地图
│   ├── package.xml
│   ├── src                           自定义场景模拟模块源代码
│   └── srv                           自定义场景模拟模块ROS中使用的自定义消息
├── doc                               说明文档
│   ├── make.bat
│   ├── Makefile
│   └── source
├── README.md
└── script                            模拟器相关脚本
    └── add_keyboard_udev             添加键盘映射脚本
```
