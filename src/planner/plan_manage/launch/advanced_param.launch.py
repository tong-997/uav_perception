import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression, TextSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ------- 定义将被使用的 LaunchConfiguration -------
    # 节点名 / 基础
    name_of_ego_planner_node = LaunchConfiguration('name_of_ego_planner_node')
    drone_id        = LaunchConfiguration('drone_id')
    obj_num_set     = LaunchConfiguration('obj_num_set')

    # 话题（可由父 launch 覆盖）
    odometry_topic    = LaunchConfiguration('odometry_topic')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic')
    depth_topic       = LaunchConfiguration('depth_topic')
    cloud_topic       = LaunchConfiguration('cloud_topic')

    # 地图尺寸
    map_size_x_ = LaunchConfiguration('map_size_x_')
    map_size_y_ = LaunchConfiguration('map_size_y_')
    map_size_z_ = LaunchConfiguration('map_size_z_')

    # 相机内参
    cx = LaunchConfiguration('cx')
    cy = LaunchConfiguration('cy')
    fx = LaunchConfiguration('fx')
    fy = LaunchConfiguration('fy')

    # 规划 / 飞行
    flight_type       = LaunchConfiguration('flight_type')
    planning_horizon  = LaunchConfiguration('planning_horizon')
    use_distinctive   = LaunchConfiguration('use_distinctive_trajs')
    max_vel           = LaunchConfiguration('max_vel')
    max_acc           = LaunchConfiguration('max_acc')

    # 航点
    point_num = LaunchConfiguration('point_num')
    point0_x = LaunchConfiguration('point0_x'); point0_y = LaunchConfiguration('point0_y'); point0_z = LaunchConfiguration('point0_z')
    point1_x = LaunchConfiguration('point1_x'); point1_y = LaunchConfiguration('point1_y'); point1_z = LaunchConfiguration('point1_z')
    point2_x = LaunchConfiguration('point2_x'); point2_y = LaunchConfiguration('point2_y'); point2_z = LaunchConfiguration('point2_z')
    point3_x = LaunchConfiguration('point3_x'); point3_y = LaunchConfiguration('point3_y'); point3_z = LaunchConfiguration('point3_z')
    point4_x = LaunchConfiguration('point4_x'); point4_y = LaunchConfiguration('point4_y'); point4_z = LaunchConfiguration('point4_z')

    # ------- 声明所有将被使用的参数（带默认值） -------
    declares = [
        # 节点名（关键：避免“does not exist”）
        DeclareLaunchArgument(
            'name_of_ego_planner_node',
            default_value=PythonExpression(["'ego_planner_' + str(", drone_id, ")"])
        ),
        DeclareLaunchArgument('drone_id',    default_value=TextSubstitution(text='0')),
        DeclareLaunchArgument('obj_num_set', default_value=TextSubstitution(text='10')),

        # 话题（默认与你当前链路一致；父 launch 会覆盖）
        DeclareLaunchArgument('odometry_topic',    default_value=TextSubstitution(text='/vins/odometry')),
        DeclareLaunchArgument('camera_pose_topic', default_value=TextSubstitution(text='/ego/camera_pose_ps')),
        #DeclareLaunchArgument('depth_topic',       default_value=TextSubstitution(text='/ego/depth_image')),
        DeclareLaunchArgument('depth_topic',       default_value=TextSubstitution(text='/unused')),
        DeclareLaunchArgument('cloud_topic',       default_value=TextSubstitution(text='/ego/points')),

        # 地图尺寸
        DeclareLaunchArgument('map_size_x_', default_value=TextSubstitution(text='50.0')),
        DeclareLaunchArgument('map_size_y_', default_value=TextSubstitution(text='25.0')),
        DeclareLaunchArgument('map_size_z_', default_value=TextSubstitution(text='2.0')),

        # 相机内参（640x480）
        DeclareLaunchArgument('cx', default_value=TextSubstitution(text='335.18186')),
        DeclareLaunchArgument('cy', default_value=TextSubstitution(text='217.7217')),
        DeclareLaunchArgument('fx', default_value=TextSubstitution(text='391.50625')),
        DeclareLaunchArgument('fy', default_value=TextSubstitution(text='395.18355')),

        # 规划 / 飞行
        DeclareLaunchArgument('flight_type',      default_value=TextSubstitution(text='2')),
        DeclareLaunchArgument('planning_horizon', default_value=TextSubstitution(text='7.5')),
        DeclareLaunchArgument('use_distinctive_trajs', default_value=TextSubstitution(text='True')),
        DeclareLaunchArgument('max_vel', default_value=TextSubstitution(text='2.0')),
        DeclareLaunchArgument('max_acc', default_value=TextSubstitution(text='6.0')),

        # 航点（给与父一致的默认）
        DeclareLaunchArgument('point_num', default_value=TextSubstitution(text='4')),
        DeclareLaunchArgument('point0_x', default_value=TextSubstitution(text='15.0')),
        DeclareLaunchArgument('point0_y', default_value=TextSubstitution(text='0.0')),
        DeclareLaunchArgument('point0_z', default_value=TextSubstitution(text='1.0')),
        DeclareLaunchArgument('point1_x', default_value=TextSubstitution(text='-15.0')),
        DeclareLaunchArgument('point1_y', default_value=TextSubstitution(text='0.0')),
        DeclareLaunchArgument('point1_z', default_value=TextSubstitution(text='1.0')),
        DeclareLaunchArgument('point2_x', default_value=TextSubstitution(text='15.0')),
        DeclareLaunchArgument('point2_y', default_value=TextSubstitution(text='0.0')),
        DeclareLaunchArgument('point2_z', default_value=TextSubstitution(text='1.0')),
        DeclareLaunchArgument('point3_x', default_value=TextSubstitution(text='-15.0')),
        DeclareLaunchArgument('point3_y', default_value=TextSubstitution(text='0.0')),
        DeclareLaunchArgument('point3_z', default_value=TextSubstitution(text='1.0')),
        DeclareLaunchArgument('point4_x', default_value=TextSubstitution(text='15.0')),
        DeclareLaunchArgument('point4_y', default_value=TextSubstitution(text='0.0')),
        DeclareLaunchArgument('point4_z', default_value=TextSubstitution(text='1.0')),
    ]

    # ------- ego_planner 节点 -------
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        output='screen',
        name=name_of_ego_planner_node,
        remappings=[
            # 统一使用传入的里程计与传感话题（不再硬编码）
            ('odom_world', odometry_topic),
            #('planning/bspline', 'planning/bspline'),
            ('planning/bspline', [TextSubstitution(text='/drone_'),
                          drone_id,
                          TextSubstitution(text='_planning/bspline')]),

            ('planning/data_display', 'planning/data_display'),
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),

            ('grid_map/odom',  odometry_topic),
            ('grid_map/pose',  camera_pose_topic),
            ('grid_map/depth', depth_topic),
            # 如需点云可解开下一行（确保上游提供 cloud_topic）
            ('grid_map/cloud', cloud_topic),

            ('grid_map/occupancy_inflate', 'grid_map/occupancy_inflate'),
            ('optimal_list', 'optimal_list'),
        ],
        parameters=[
            # FSM
            {'fsm/flight_type': flight_type},
            {'fsm/thresh_replan_time': 1.0},
            {'fsm/thresh_no_replan_meter': 1.0},
            {'fsm/planning_horizon': planning_horizon},
            {'fsm/planning_horizen_time': 3.0},
            {'fsm/emergency_time': 1.0},
            {'fsm/realworld_experiment': False},
            {'fsm/fail_safe': True},

            {'fsm/waypoint_num': point_num},
            {'fsm/waypoint0_x': point0_x}, {'fsm/waypoint0_y': point0_y}, {'fsm/waypoint0_z': point0_z},
            {'fsm/waypoint1_x': point1_x}, {'fsm/waypoint1_y': point1_y}, {'fsm/waypoint1_z': point1_z},
            {'fsm/waypoint2_x': point2_x}, {'fsm/waypoint2_y': point2_y}, {'fsm/waypoint2_z': point2_z},
            {'fsm/waypoint3_x': point3_x}, {'fsm/waypoint3_y': point3_y}, {'fsm/waypoint3_z': point3_z},
            {'fsm/waypoint4_x': point4_x}, {'fsm/waypoint4_y': point4_y}, {'fsm/waypoint4_z': point4_z},

            # grid_map
            # 地面/顶棚与可视截断高度
            {'grid_map/resolution': 0.2},
            {'grid_map/map_size_x': map_size_x_},
            {'grid_map/map_size_y': map_size_y_},
            {'grid_map/map_size_z': map_size_z_},
            {'grid_map/local_update_range_x': 5.5},
            {'grid_map/local_update_range_y': 5.5},
            {'grid_map/local_update_range_z': 4.5},
            #等整个链路稳定后，再把 obstacles_inflation 与 p_occ 慢慢拉回去。
            {'grid_map/obstacles_inflation': 0.25},#0.4
            {'grid_map/local_map_margin': 10},
            {'grid_map/ground_height': -0.15},#-0.01

            {'grid_map/cx': cx},
            {'grid_map/cy': cy},
            {'grid_map/fx': fx},
            {'grid_map/fy': fy},

            {'grid_map/use_depth_filter': False},# 点云模式关闭深度滤波
            {'grid_map/depth_filter_tolerance': 0.20},#0.15
            {'grid_map/depth_filter_maxdist': 6.0},#5.0
            {'grid_map/depth_filter_mindist': 0.3},#0.2
            {'grid_map/depth_filter_margin': 2},
            {'grid_map/k_depth_scaling_factor': 1.0},
            {'grid_map/skip_pixel': 4},
            # 占用概率与膨胀
            #等整个链路稳定后，再把 obstacles_inflation 与 p_occ 慢慢拉回去。
            {'grid_map/p_hit': 0.60},#0.65
            {'grid_map/p_miss': 0.40},#0.35
            {'grid_map/p_min': 0.12},
            {'grid_map/p_max': 0.90},
            {'grid_map/p_occ': 0.75},#0.80
            #观测模型与射线长度，微调
            {'grid_map/min_ray_length': 0.2},#0.1
            {'grid_map/max_ray_length': 6.0},#4.5
            #地面/顶棚与可视截断高度
            {'grid_map/virtual_ceil_height': 3.2},#2.9
            {'grid_map/visualization_truncate_height': 2.2},#1.8
            {'grid_map/show_occ_time': False},
            {'grid_map/pose_type': 0},# ★ 0=用 odom 位姿（点云常用）
            #修改这个坐标系id,让ego统一在vins的world下工作，不出现map/world混用
            {'grid_map/frame_id': "world"},

            # manager / optimization / bspline / prediction
            {'manager/max_vel': max_vel},
            {'manager/max_acc': max_acc},
            {'manager/max_jerk': 4.0},
            {'manager/control_points_distance': 0.4},
            {'manager/feasibility_tolerance': 0.05},
            {'manager/planning_horizon': planning_horizon},
            {'manager/use_distinctive_trajs': use_distinctive},
            {'manager/drone_id': drone_id},

            {'optimization/lambda_smooth': 1.0},
            {'optimization/lambda_collision': 0.5},
            {'optimization/lambda_feasibility': 0.1},
            {'optimization/lambda_fitness': 1.0},
            {'optimization/dist0': 0.5},
            {'optimization/swarm_clearance': 0.5},
            {'optimization/max_vel': max_vel},
            {'optimization/max_acc': max_acc},

            {'bspline/limit_vel': max_vel},
            {'bspline/limit_acc': max_acc},
            {'bspline/limit_ratio': 1.1},

            {'prediction/obj_num': obj_num_set},
            {'prediction/lambda': 1.0},
            {'prediction/predict_rate': 1.0}
        ]
    )

    # ------- 组装与返回 -------
    ld = LaunchDescription()
    for d in declares:
        ld.add_action(d)
    ld.add_action(ego_planner_node)
    return ld

