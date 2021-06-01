
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from scripts import GazeboRosPaths
import sys
import xacro

def generate_launch_description():


    # Starting poses for robots
    robots     = [
                    ('robot_00000001',     (    0.8,      2.0,      1.9   )),
                    ('robot_00000002',     (    0.75,      1.4,      0.1   )),
                    ('robot_00000003',     (    0.82,      0.92,      0.3   )),
                    ('robot_00000004',     (    0.81,      0.26,      0.7   )),
                    ('robot_00000005',     (    0.77,      -0.4,      1.2   )),
    ]

    # Package directories
    pkg_share       = get_package_share_directory('dots_sim')
    pkg_controller  = get_package_share_directory('dots_example_controller')


    # Launch config variables
    use_sim_time    = LaunchConfiguration('use_sim_time', default='True')
    use_rviz        = LaunchConfiguration('use_rviz', default='False')

    declare_use_sim_time    = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_use_rviz        = DeclareLaunchArgument('use_rviz', default_value='false')

    # Build the launch description
    ld = LaunchDescription()

    # Spawn the robots serially
    # The individual robot launch will create the robot specific nodes, but it seems that
    # spawning multiple models simultaneously in Gazebo hits limits in Gazebo, fastrtps, or
    # both. To get round this, we spawn all the robots from a single script that does them
    # serially
    robot_string = ' '.join(['%s %f %f %f ' % (r[0], r[1][0], r[1][1], r[1][2]) for r in robots])
    arg = ['./spawn_multiple.sh'] + robot_string.split()
    print(arg)

    ld.add_action(ExecuteProcess(
        cwd         = os.path.join(pkg_share, 'launch'),
        output      = 'screen',
        cmd         = arg
    ))

    for r in robots:
        print('robot:%s pose:%s' % (r[0],r[1]))
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'controller.launch.py')),
            launch_arguments    = { 'robot_name'    : PythonExpression(['"', r[0], '"']),
                                    'robot_pose'    : '%f,%f,%f' % (r[1][0], r[1][1], r[1][2]),
                                    'use_sim_time'  : use_sim_time,
            }.items()
        ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource( os.path.join(pkg_share, 'launch', 'gazebo_rviz.launch.py')),
        launch_arguments    = {'use_rviz'   : use_rviz}.items()
    ))

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)

    return ld
    

