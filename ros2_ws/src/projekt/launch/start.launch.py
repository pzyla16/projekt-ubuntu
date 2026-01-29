from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():


    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )

    camera_node = Node(
        package='image_tools',
        executable='cam2image',
        name='virtual_camera',
        parameters=[{
            'burger_mode': True
        }]
    )


    click_drive_node = Node(
        package='projekt',
        executable='click_node',
        name='click_node',
        parameters=[{
            'image_topic': '/image',
            'cmd_vel_topic': '/turtle1/cmd_vel',
            'linear_speed': 1.0
        }],
        output='screen'
    )

    return LaunchDescription([
        turtlesim_node,
        camera_node,
        click_drive_node
    ])
