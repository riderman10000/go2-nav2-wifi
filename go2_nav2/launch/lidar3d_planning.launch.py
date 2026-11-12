"""Nav2 online plan preview from current Point-LIO scans; no motion nodes."""
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
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
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    with open(pkg + '/config/go2_nav2_minimal.yaml') as stream:
        planner = yaml.safe_load(stream)['planner_server']
    planner['ros__parameters']['use_sim_time'] = sim
    planner['ros__parameters']['GridBased']['allow_unknown'] = False
    with open(pkg + '/config/lidar3d_planner.yaml') as stream:
        data = yaml.safe_load(stream)
    data['global_costmap']['global_costmap']['ros__parameters']['use_sim_time'] = sim
    data['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['lidar']['topic'] = '/lidar3d/obstacle_points'
    if os.environ.get('GO2_LIDAR3D_NAV') == '1':
        cost = data['global_costmap']['global_costmap']['ros__parameters']
        cost.pop('robot_radius', None)
        with open(pkg + '/config/go2_footprint.yaml') as stream:
            cost.update(yaml.safe_load(stream))
        # State Lattice collision-checks the complete oriented polygon. Smac2D
        # only searches XY and cannot model a rectangular robot turning in a gap.
        cost['resolution'] = 0.05
        cost['obstacle_layer']['lidar']['topic'] = '/lidar3d/obstacle_points'
        planner['ros__parameters']['GridBased'] = {
            'plugin': 'nav2_smac_planner/SmacPlannerLattice',
            'lattice_filepath': get_package_share_directory('nav2_smac_planner') +
                '/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json',
            # Sparse 3D rays leave unknown gaps even on the observed floor.
            # Explore through unknown cells; retain full polygon checks against
            # marked obstacles and the live local obstacle costmap.
            'allow_unknown': True, 'allow_reverse_expansion': False,
            'tolerance': 0.25, 'max_iterations': 1000000,
            'max_on_approach_iterations': 1000, 'max_planning_time': 3.0,
            'smooth_path': False, 'cache_obstacle_heuristic': False,
            'rotation_penalty': 5.0,
            # go2-clearance: weigh inflation cost more (Smac default 2.0), so
            # paths keep to the middle of gaps instead of grazing walls.
            'cost_penalty': 3.0,
        }
        # go2-clearance: a narrower band (was 0.55) with a slower fall-off (was
        # 3.0): more of a doorway is low-cost, the middle still cheapest.
        cost['inflation_layer']['inflation_radius'] = 0.45
        cost['inflation_layer']['cost_scaling_factor'] = 2.0
        cost['footprint_padding'] = 0.05  # go2-clearance: gait sway (was 0.03)
        cost['transform_tolerance'] = 0.3
        cost['obstacle_layer']['lidar']['expected_update_rate'] = 2.0  # go2-wifi-lidar was: cost['obstacle_layer']['lidar']['expected_update_rate'] = 0.8
        if os.environ.get('GO2_FLOOR_CLEAR', '1') != '0':  # go2-clearing: 0 = off, for A/B tests
            add_floor_clearing(cost)
    data['planner_server'] = planner
    fd, path = tempfile.mkstemp(prefix='go2-lidar3d-planner-', suffix='.yaml')
    with os.fdopen(fd, 'w') as stream:
        yaml.safe_dump(data, stream)
    def cleanup(_context):
        os.unlink(path)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[path]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_lidar3d_planning', output='screen', parameters=[{
                 'use_sim_time': sim, 'autostart': True, 'node_names': ['planner_server'],
                 'bond_timeout': 10.0}]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('use_sim_time', default_value='false'),
                              OpaqueFunction(function=configure)])
