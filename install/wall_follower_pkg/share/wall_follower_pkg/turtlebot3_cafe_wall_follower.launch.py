import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    gazebo_launch_dir = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_launch_dir = get_package_share_directory('turtlebot3_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world_path = os.path.join(turtlebot3_gazebo_launch_dir, 'worlds', 'cafe.world')

    set_turtlebot3_model_env = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle')

    delete_existing_robot = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/delete_entity', 'gazebo_msgs/srv/DeleteEntity', '{name: "turtlebot3"}'],
        shell=True
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_launch_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_launch_dir, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x': '0', 'y': '0', 'z': '0'}.items()
    )

    wall_follower_node = Node(
        package='wall_follower_pkg',
        executable='wall_follower_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    reset_simulation = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/reset_simulation', 'std_srvs/srv/Empty'],
        shell=True
    )

    reset_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[reset_simulation]
        )
    )

    return LaunchDescription([
        set_turtlebot3_model_env,
        delete_existing_robot,
        gazebo,
        robot_state_publisher,
        spawn_turtlebot,
        wall_follower_node,
        reset_handler
    ])
