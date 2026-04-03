from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_iris = get_package_share_directory("iris")
    pkg_root = os.path.join(pkg_iris, "..", "..", "..", "..")  # workspace root

    return LaunchDescription([
        DeclareLaunchArgument(
            "iris_config_path",
            default_value=os.path.join(pkg_iris, "..", "config", "iris_mono_kitti.yaml"),
        ),
        DeclareLaunchArgument(
            "vslam_config_path",
            default_value=os.path.join(
                pkg_iris, "..", "openvslam_bridge", "3rd", "openvslam",
                "example", "kitti", "KITTI_mono_00-02.yaml"),
        ),
        DeclareLaunchArgument("pcd_path", default_value=os.path.join(pkg_root, "kitti_00.pcd")),
        DeclareLaunchArgument("vocab_file", default_value=os.path.join(pkg_root, "orb_vocab.dbow2")),
        DeclareLaunchArgument("image_topic_name0", default_value="/image_raw0"),
        DeclareLaunchArgument("is_image_compressed", default_value="true"),
        DeclareLaunchArgument("is_image_color", default_value="false"),
        DeclareLaunchArgument("keyframe_recollection", default_value="20"),

        Node(
            package="iris",
            executable="iris_node",
            name="iris_node",
            output="screen",
            parameters=[{
                "iris_config_path": LaunchConfiguration("iris_config_path"),
                "pcd_path": LaunchConfiguration("pcd_path"),
            }],
        ),

        Node(
            package="openvslam_bridge",
            executable="openvslam_bridge_node",
            name="openvslam_bridge_node",
            output="screen",
            parameters=[{
                "vslam_config_path": LaunchConfiguration("vslam_config_path"),
                "vocab_path": LaunchConfiguration("vocab_file"),
                "image_topic_name0": LaunchConfiguration("image_topic_name0"),
                "is_image_compressed": LaunchConfiguration("is_image_compressed"),
                "is_image_color": LaunchConfiguration("is_image_color"),
                "keyframe_recollection": LaunchConfiguration("keyframe_recollection"),
            }],
        ),
    ])
