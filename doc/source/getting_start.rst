


====================
Getting Start
====================

现在让我们编译安装 `Autolabor_simulation`_ 并且运行第一个Hello World吧！

.. _Autolabor_simulation: https://github.com/gsc07/autolabor_simulation

Build & Installation
=====================

`Autolabor_simulation`_ 仓库由若干个 `Ros Package`_ 组成，仓库下的每个package
有自己的配置文件（package.xml），可以直接将 `Autolabor_simulation`_ 克隆到你
的workspace的src文件夹下，利用catkin_make编译autolabor_simulation.

创建你的 `catkin_workspace`_

.. code-block:: bash

    # Create catkin_workspace
    source /opt/ros/kinetic/setup.bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make

下载并编译

.. code-block:: bash

    # Clone Autolabor_simulation to your workspace
    cd ~/catkin_ws/src/
    git clone https://github.com/gsc07/autolabor_simulation.git

    # Build source
    cd ..
    catkin_make
    source devel/setup.bash

如果希望Ros环境中默认包含autolabor_simulation，可以使用如下命令将workspace设为默认包含

.. code-block:: bash

    # Source workspace for every terminal
    source ~/catkin_ws/devel/setup.bash


.. _Autolabor_simulation: https://github.com/gsc07/autolabor_simulation
.. _Ros Package: http://wiki.ros.org/Packages
.. _catkin_workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace


A Hello World
==================

TODO
