通过终端修改参数值
运行的参数功能节点，打开一个新终端，使用如下命令查询当前所有参数列表。
	ros2 param list
在列表中可以看到my_parameter这个参数，接下来就可以通过如下命令行来修改这个参数的值了。
	ros2 param set /parameter_node my_parameter earth
这样，我们就将my_parameter参数的值修改为“earth”了
