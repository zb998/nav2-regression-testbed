"""
真实 Gazebo+Nav2 的"骨架"，路径都是占位符，需要你改成自己的。框架只认 4 个接口，
所以只要你自己的 bring-up 能起 Nav2，就能替换。

@author: zbb
@date: 2026/11/07
@version: v1.0XXX
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明启动参数（相当于在终端敲 ros2 launch xxx.launch.py headless:=true map:=...）
    # headless: 是否无头运行（不开 Gazebo 的 3D 图形界面），默认 true 适合服务器跑 CI
    headless = DeclareLaunchArgument('headless', default_value='true',
        description='Run Gazebo without a GUI (for CI).')

    # map: 地图文件的路径（default_value 需要改成自己电脑上的真实绝对路径）
    map_file = DeclareLaunchArgument('map', default_value='<path/to/map.yaml>')
    # world: Gazebo 仿真世界文件路径
    world_file = DeclareLaunchArgument('world', default_value='<path/to/world.sdf>')

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', LaunchConfiguration('world'),
             '--headless-rendering'], output='screen')

    # 因为 Gazebo 用的是自己的底层协议，听不懂 ROS 2 的 msg，必须中间加一个桥接
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  output='screen',
                  parameters=[{'config_file': '<path/to/bridge.yaml>'}])

    # 直接引入官方 nav2_bringup 的启动文件（不用我们自己手写一堆节点）
    nav2 = IncludeLaunchDescription(
        # FindPackageShare 会自动找到 nav2_bringup 包在系统里的安装路径
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])),
        # 把我们声明的 map 路径，以及指定的参数文件传进 Nav2
        launch_arguments={'map': LaunchConfiguration('map'),
                          'params_file': '<path/to/nav2_params.yaml>'}.items())

     # 按照顺序把参数、Gazebo、桥接、Nav2 一起启动
    return LaunchDescription([headless, map_file, world_file, gz_sim, bridge, nav2])