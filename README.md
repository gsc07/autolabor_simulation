# autolabor_simulation

Build & Installation
--------------------

[Autolabor\_simulation](https://github.com/gsc07/autolabor_simulation)
仓库由若干个 [Ros Package](http://wiki.ros.org/Packages)
组成，仓库下的每个package 有自己的配置文件（package.xml），可以直接将
[Autolabor\_simulation](https://github.com/gsc07/autolabor_simulation)
克隆到你
的workspace的src文件夹下，利用catkin\_make编译autolabor\_simulation.

创建你的
[catkin\_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

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
