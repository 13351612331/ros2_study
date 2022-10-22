from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim' #Node name after startup
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        #mimic节点通过订阅turtle1的位置，转换成对turtle2的速度指令发布出去，最后应该可以达到让turtle2模仿turtle1完成同样的运动
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose','/turtlesim1/turtle1/pose'), #输入话题从/input/pose重映射为/turtlesim1/turtle1/pose
                ('/output/cmd_vel','/turtlesim2/turtle1/cmd_vel'), #输出话题从/output/cmd_vel重映射为/turtlesim2/turtle1/cmd_vel
            ]
        )
    ])