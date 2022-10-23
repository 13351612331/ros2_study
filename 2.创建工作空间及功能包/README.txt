** 1.开发每一个项目的工作空间，最好是独立创建一个文件夹，文件夹的名字可以自由定义，比如我们来创建一个开发用的空间空间，就叫做dev_ws：
	mkdir -p ~/dev_ws/src
	cd ~/dev_ws/src
	
** 2.可以使用如下语法来创建功能包了（以下二选一）：   
   **2.1 CMake包：

	ros2 pkg create --build-type ament_cmake <package_name>

   **2.2 Python包：

	ros2 pkg create --build-type ament_python <package_name>

      创建功能包的指令还允许设置节点名，自动生成一个helloworld的例程代码（以下二选一）。   
      CMake包：

	ros2 pkg create --build-type ament_cmake --node-name my_node my_package

      Python包：

	ros2 pkg create --build-type ament_python --node-name my_node my_package

** 3.编译功能包
	接下来进入编译流程，一定要将终端cd到dev_ws的路径下来：

		cd ~/dev_ws

  	然后就可以编译啦：

		colcon build

  	这个命令会编译工作空间中的所有功能包，如果只想编译某一个包的话，可以这样：

		colcon build --packages-select my_package
		
** 4.设置环境变量
	打开一个新的终端，运行下工作空间的环境变量，这样才能让该终端找到新创建的包：

		. install/setup.bash

  	接下来就可以在该终端中愉快的运行新建的功能包了。
  	
** 5.运行节点
	在终端中使用如下命令即可运行新建功能包的节点啦：

		ros2 run my_package my_node
