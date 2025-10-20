from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='topic_exam',
            executable='topic_exam_pub',
            name='my_topic_pub_1',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='topic_exam',
            executable='topic_exam_sub',
            name='my_topic_sub_1',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='topic_exam',
            executable='topic_exam_pub',
            name='my_topic_pub_2',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='topic_exam',
            executable='topic_exam_sub',
            name='my_topic_sub_2',
            output='screen',
        ),
    ])