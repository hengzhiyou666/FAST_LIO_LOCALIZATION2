from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_path = get_package_share_directory("fast_lio_localization")
    default_config_path = os.path.join(package_path, "config")
    default_rviz_config_path = os.path.join(package_path, "rviz", "fastlio_localization.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_path = LaunchConfiguration("config_path")
    config_file = LaunchConfiguration("config_file")
    rviz_use = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    pcd_map_topic = LaunchConfiguration("pcd_map_topic")
    pcd_map_path = LaunchConfiguration("map")
    gps_topic = LaunchConfiguration("gps_topic")

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        "config_path", default_value=default_config_path, description="Yaml config file path"
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        "config_file", default_value="mid360.yaml", description="Config file"
    )
    declare_rviz_cmd = DeclareLaunchArgument("rviz", default_value="true", description="Use RViz to monitor results")

    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        "rviz_cfg", default_value=default_rviz_config_path, description="RViz config file path"
    )

    declare_map_path = DeclareLaunchArgument("map", default_value="", description="Path to PCD map file")
    declare_pcd_map_topic = DeclareLaunchArgument(
        "pcd_map_topic", default_value="/map", description="Topic to publish PCD map"
    )
    declare_gps_topic = DeclareLaunchArgument(
        "gps_topic", default_value="/gps_rtk_fix", description="NavSatFix topic for GPS"
    )
    # Load parameters from yaml file

    fast_lio_node = Node(
        package="fast_lio_localization",
        executable="fastlio_mapping",
        parameters=[PathJoinSubstitution([config_path, config_file]), {"use_sim_time": use_sim_time}],
        output="screen",
    )
    # Global localization node
    global_localization_node = Node(
        package="fast_lio_localization",
        executable="global_localization.py",
        name="global_localization",
        output="screen",
        parameters=[{"map_voxel_size": 0.4,
                     "scan_voxel_size": 0.1,
                     "freq_localization": 1.0,  # 提高到1.0Hz，加快收敛速度
                     "freq_global_map": 0.25,
                     "localization_threshold": 0.8,
                     "fov": 6.28319,
                     "fov_far": 300,
                     "pcd_map_path": pcd_map_path,
                     "pcd_map_topic": pcd_map_topic,
                     "icp_max_distance_multiplier": 2.0,  # 增大ICP最大距离，允许更大初始偏差
                     "icp_max_iteration": 30,  # 增加迭代次数，提高收敛精度
                     "use_sim_time": use_sim_time}],
    )

    # Transform fusion node
    transform_fusion_node = Node(
        package="fast_lio_localization",
        executable="transform_fusion.py",
        name="transform_fusion",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # PCD to PointCloud2 publisher
    # Note: pcd_to_pointcloud publishes to 'cloud_pcd' topic by default
    # We remap it to the desired topic name
    pcd_publisher_node = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="map_publisher",
        output="screen",
        parameters=[{"file_name": pcd_map_path,
                     "tf_frame": "map",
                    "period_ms_": 500}],
        remappings=[
            ("cloud_pcd", pcd_map_topic),
        ]
    )

    # GPS path node (convert NavSatFix -> map 平面坐标 -> Path)
    gps_path_node = Node(
        package="fast_lio_localization",
        executable="gps_path_node.py",
        name="gps_path_node",
        output="screen",
        parameters=[{"gps_topic": gps_topic,
                     "use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2", 
        executable="rviz2", 
        arguments=["-d", rviz_cfg], 
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz_use)
    )

    # Odometry 四元数打印节点（在新终端中运行，1Hz 频率）
    # 使用 ExecuteProcess 在新终端窗口中运行
    workspace_path = os.path.expanduser("~/heng_location_20260128_ws")
    odom_quaternion_printer_node = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--title=Odometry Quaternion Monitor (1Hz)",
            "--",
            "bash",
            "-c",
            f"cd {workspace_path} && source install/setup.bash && ros2 run fast_lio_localization print_odom_quaternion; exec bash"
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_map_path)
    ld.add_action(declare_pcd_map_topic)
    ld.add_action(declare_gps_topic)

    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)
    ld.add_action(global_localization_node)
    ld.add_action(transform_fusion_node)
    ld.add_action(pcd_publisher_node)
    ld.add_action(gps_path_node)
    ld.add_action(odom_quaternion_printer_node)

    return ld
