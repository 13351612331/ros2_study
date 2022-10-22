cd launch
ros2 launch turtlesim_mimic_launch.py
很快可以看到两个小海龟仿真窗口启动啦
为了看到turtle2是否会模仿turtle1，我们需要在让turtle1动起来：
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
为了看到更明确的节点关系，我们可以使用之前介绍的rqt_graph工具来观测下系统