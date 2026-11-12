"""Supervised level-floor controller, added to an existing 3D planning session.

Robot transport is started separately after validation.
"""
import copy
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node


def add_floor_clearing(cost):
    """go2-clearing: an erase-only lidar source that keeps the floor hits.

    The lidar source keeps points 0.15-1.6 m high, and Nav2 erases cells only
    along rays to points it kept. Rays that hit the floor, which are most of
    them near the dog (the L1 looks forward and down), were thrown away, so a
    spot where a person had stood was never erased.

    It reads the raw /lidar3d/registered cloud. By the time this runs the
    lidar source has been switched to /lidar3d/obstacle_points, which the
    floor filter has already stripped of floor hits, so copying that topic
    left nothing to clear with and the floor in front of the dog stayed
    unknown.
    """
    layer = cost['obstacle_layer']
    layer['observation_sources'] += ' floor_clear'
    lidar = layer['lidar']
    layer['floor_clear'] = {
        'topic': '/lidar3d/registered', 'sensor_frame': lidar['sensor_frame'],
        'data_type': 'PointCloud2', 'marking': False, 'clearing': True,
        'min_obstacle_height': -2.0, 'max_obstacle_height': 2.0,
        'raytrace_min_range': lidar['raytrace_min_range'],
        'raytrace_max_range': lidar['raytrace_max_range'],
        'expected_update_rate': 0.0, 'observation_persistence': 0.0,
    }


def configure(context):
    pkg = get_package_share_directory('go2_nav2')
    with open(pkg + '/config/go2_nav2_minimal.yaml') as stream:
        controller = yaml.safe_load(stream)['controller_server']
    params = controller['ros__parameters']
    params.update(odom_topic='/lidar3d/body_odom', controller_frequency=10.0,
                  transform_tolerance=0.3, failure_tolerance=0.0,
                  min_x_velocity_threshold=0.01, min_y_velocity_threshold=0.01,
                  min_theta_velocity_threshold=0.02)
    params['progress_checker']['movement_time_allowance'] = 35.0
    params['general_goal_checker']['xy_goal_tolerance'] = 0.25
    # Regulated Pure Pursuit behind the rotation shim. RPP follows the path
    # geometrically and does not build its command window from measured odom
    # velocity, so noisy Point-LIO-derived twist cannot make it jitter like DWB.
    params['FollowPath'] = {
        'plugin': 'nav2_rotation_shim_controller::RotationShimController',
        'primary_controller': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        # Wider hysteresis: rotate in place only for large errors, hand back early.
        'angular_dist_threshold': 0.8, 'angular_disengage_threshold': 0.2,
        # Slow turns: gyro-only Point-LIO lost its gravity direction (map tilted
        # 17.7 deg) in a 230 deg walking turn at 25-28 deg/s (run 2026-10-29 18:19).
        'forward_sampling_distance': 0.5, 'rotate_to_heading_angular_vel': 0.3,
        'max_angular_accel': 0.6, 'simulate_ahead_time': 1.0,
        'rotate_to_goal_heading': False, 'closed_loop': False,
        'desired_linear_vel': 0.30,
        'lookahead_dist': 0.6, 'min_lookahead_dist': 0.4, 'max_lookahead_dist': 0.9,
        'lookahead_time': 1.5, 'use_velocity_scaled_lookahead_dist': False,
        'transform_tolerance': 0.3,
        'min_approach_linear_velocity': 0.08, 'approach_velocity_scaling_dist': 0.6,
        'use_collision_detection': True, 'max_allowed_time_to_collision_up_to_carrot': 1.0,
        'use_regulated_linear_velocity_scaling': True,
        'use_cost_regulated_linear_velocity_scaling': False,
        'regulated_linear_scaling_min_radius': 0.6, 'regulated_linear_scaling_min_speed': 0.12,
        'use_rotate_to_heading': False,  # the shim handles rotation
        'allow_reversing': False, 'max_robot_pose_search_dist': 3.0,
    }
    with open(pkg + '/config/lidar3d_planner.yaml') as stream:
        local = copy.deepcopy(yaml.safe_load(stream)['global_costmap']['global_costmap'])
    cost = local['ros__parameters']
    cost.update(width=6, height=6, resolution=0.05, update_frequency=5.0,
                track_unknown_space=False,
                transform_tolerance=0.3)
    with open(pkg + '/config/go2_footprint.yaml') as stream:
        cost.update(yaml.safe_load(stream))
    cost.pop('robot_radius', None)
    cost['obstacle_layer']['lidar']['expected_update_rate'] = 2.0  # go2-wifi-lidar was: cost['obstacle_layer']['lidar']['expected_update_rate'] = 0.8
    cost['obstacle_layer']['lidar']['topic'] = '/lidar3d/obstacle_points'
    # go2-clearance: same band and padding as the planner's costmap.
    cost['inflation_layer']['inflation_radius'] = 0.45
    cost['inflation_layer']['cost_scaling_factor'] = 2.0
    cost['footprint_padding'] = 0.05
    if os.environ.get('GO2_FLOOR_CLEAR', '1') != '0':  # go2-clearing: 0 = off, for A/B tests
        add_floor_clearing(cost)
    fd, path = tempfile.mkstemp(prefix='go2-lidar3d-controller-', suffix='.yaml')
    with os.fdopen(fd, 'w') as stream:
        yaml.safe_dump({'controller_server': controller, 'local_costmap': {'local_costmap': local}}, stream)

    def cleanup(_context):
        os.unlink(path)
        return []

    with open(pkg + '/config/lidar3d_robot_viz.yaml') as stream:
        odom = yaml.safe_load(stream)['go2_odom_tf']['ros__parameters']
    odom.update(publish_odom=True, derive_planar_twist=True, publish_tf=False,
                odom_out_topic='/lidar3d/body_odom')
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='go2_nav2', executable='go2_odom_tf', name='lidar3d_body_odom', parameters=[odom]),
        # Controller output goes through the smoother; only the smoother publishes /cmd_vel.
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[path], remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             output='screen',
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
             parameters=[{
                 'smoothing_frequency': 20.0, 'scale_velocities': True,
                 'feedback': 'OPEN_LOOP',  # never trust the noisy measured twist
                 # Turn rate capped at 0.35 rad/s (20 deg/s): see the shim note above.
                 'max_velocity': [0.30, 0.0, 0.35], 'min_velocity': [-0.30, 0.0, -0.35],
                 'max_accel': [0.5, 0.0, 0.5], 'max_decel': [-0.5, 0.0, -0.5],
                 # Go2 steps in place on tiny commands; zero them instead.
                 'deadband_velocity': [0.03, 0.0, 0.08],
                 'velocity_timeout': 1.0}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_lidar3d_controller', parameters=[{
                 'autostart': True, 'node_names': ['controller_server', 'velocity_smoother'], 'bond_timeout': 4.0}]),
        Node(package='go2_nav2', executable='go2_goal_pose_nav', name='go2_goal_pose_nav',
             output='screen', parameters=[{'map_frame': 'lidar3d_map', 'pre_rotate_enable': False}]),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=configure)])
