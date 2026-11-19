from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'start_side',
            default_value='left',
            description="Starting entrance: 'left' (turns right) or 'right' (turns left)",
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='',
            description='Optional robot name override (leave empty to auto-discover)',
        ),
        DeclareLaunchArgument(
            'long_distance',
            default_value='1.5',
            description='Distance in meters for the 1st and 3rd straight legs',
        ),
        DeclareLaunchArgument(
            'short_distance',
            default_value='1.0',
            description='Distance in meters for the 2nd (shorter) straight leg',
        ),
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.25',
            description='Forward linear.x speed',
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.8',
            description='Turn angular.z speed magnitude',
        ),
        DeclareLaunchArgument(
            'drift_angular_trim',
            default_value='-0.05',
            description='Angular trim during straights (negative pulls right to counter left drift)',
        ),
        DeclareLaunchArgument(
            'drift_strafe_trim',
            default_value='0.0',
            description='Optional linear.y trim during straights',
        ),
        DeclareLaunchArgument(
            'distance_scale',
            default_value='1.0',
            description='Multiplier to calibrate straight-line distance',
        ),
        DeclareLaunchArgument(
            'turn_scale',
            default_value='1.5',
            description='Multiplier to calibrate 90-deg turns (1.5 offsets bridge /1.5 divisor)',
        ),
        DeclareLaunchArgument(
            'pause_duration',
            default_value='0.4',
            description='Stop/settle time in seconds between segments',
        ),
        DeclareLaunchArgument(
            'start_delay',
            default_value='2.0',
            description='Seconds to wait after topic discovery before moving',
        ),
    ]

    fallback_node = Node(
        package='moorebot_race',
        executable='race_fallback_node',
        name='moorebot_race_fallback',
        output='screen',
        parameters=[{
            'start_side': LaunchConfiguration('start_side'),
            'robot_name': LaunchConfiguration('robot_name'),
            'long_distance': ParameterValue(LaunchConfiguration('long_distance'), value_type=float),
            'short_distance': ParameterValue(LaunchConfiguration('short_distance'), value_type=float),
            'linear_speed': ParameterValue(LaunchConfiguration('linear_speed'), value_type=float),
            'angular_speed': ParameterValue(LaunchConfiguration('angular_speed'), value_type=float),
            'drift_angular_trim': ParameterValue(LaunchConfiguration('drift_angular_trim'), value_type=float),
            'drift_strafe_trim': ParameterValue(LaunchConfiguration('drift_strafe_trim'), value_type=float),
            'distance_scale': ParameterValue(LaunchConfiguration('distance_scale'), value_type=float),
            'turn_scale': ParameterValue(LaunchConfiguration('turn_scale'), value_type=float),
            'pause_duration': ParameterValue(LaunchConfiguration('pause_duration'), value_type=float),
            'start_delay': ParameterValue(LaunchConfiguration('start_delay'), value_type=float),
        }],
    )

    return LaunchDescription(args + [fallback_node])
