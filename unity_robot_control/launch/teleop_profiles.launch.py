import os
import re
from typing import Any, Dict, List, Optional

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_PLACEHOLDER_RE = re.compile(r'^\$\{([A-Za-z0-9_]+)\}$')


def _load_config(path: str) -> Dict[str, Any]:
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def _resolve_value(context, value: Any) -> Any:
    if isinstance(value, str):
        match = _PLACEHOLDER_RE.match(value.strip())
        if match:
            return LaunchConfiguration(match.group(1)).perform(context)
        return value
    if isinstance(value, list):
        return [_resolve_value(context, v) for v in value]
    if isinstance(value, dict):
        return {k: _resolve_value(context, v) for k, v in value.items()}
    return value


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _get_launch_source(package: str, launch_file: str):
    pkg_share = get_package_share_directory(package)
    launch_path = os.path.join(pkg_share, 'launch', launch_file)
    _, ext = os.path.splitext(launch_path)
    if ext == '.py':
        return PythonLaunchDescriptionSource(launch_path)
    return AnyLaunchDescriptionSource(launch_path)


def _build_launch_arguments(context, args: Dict[str, Any], omit_empty: List[str]) -> Dict[str, Any]:
    resolved = _resolve_value(context, args or {})
    launch_args: Dict[str, Any] = {}
    for key, val in resolved.items():
        if key in omit_empty and (val is None or str(val).strip() == ''):
            continue
        launch_args[key] = val
    return launch_args


def _build_node(context, entry: Dict[str, Any]):
    params = _resolve_value(context, entry.get('parameters', {}))
    if isinstance(params, dict):
        params = [params]

    remappings = _resolve_value(context, entry.get('remappings', []))
    if isinstance(remappings, dict):
        remappings = list(remappings.items())

    return Node(
        package=entry['package'],
        executable=entry['executable'],
        name=entry.get('name'),
        namespace=entry.get('namespace'),
        parameters=params if params else None,
        remappings=remappings if remappings else None,
        arguments=_resolve_value(context, entry.get('arguments', [])) or None,
        output=entry.get('output', 'screen'),
    )


def _build_include(context, entry: Dict[str, Any]):
    omit_empty = entry.get('omit_empty_args', [])
    launch_args = _build_launch_arguments(context, entry.get('args', {}), omit_empty)
    return IncludeLaunchDescription(
        _get_launch_source(entry['package'], entry['launch']),
        launch_arguments=launch_args.items(),
    )


def _build_execute(context, entry: Dict[str, Any]):
    cmd = _resolve_value(context, entry.get('cmd', []))
    if isinstance(cmd, str):
        cmd = [cmd]
    return ExecuteProcess(
        cmd=cmd,
        output=entry.get('output', 'screen'),
        shell=bool(entry.get('shell', False)),
    )


def _launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config').perform(context)
    profile_name = LaunchConfiguration('profile').perform(context)
    cfg = _load_config(config_path)
    profiles = cfg.get('profiles', {})
    profile = profiles.get(profile_name)
    if not profile:
        return [LogInfo(msg=f"[unity_robot_control] profile not found: {profile_name}")]

    actions = []
    for entry in profile.get('actions', []):
        enabled = _resolve_value(context, entry.get('enabled', True))
        if not _as_bool(enabled):
            continue
        entry_type = entry.get('type')
        if entry_type == 'node':
            actions.append(_build_node(context, entry))
        elif entry_type == 'include':
            actions.append(_build_include(context, entry))
        elif entry_type == 'execute':
            actions.append(_build_execute(context, entry))
        else:
            actions.append(LogInfo(msg=f"[unity_robot_control] unknown action type: {entry_type}"))

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('unity_robot_control')
    default_config = os.path.join(pkg_share, 'config', 'launch_profiles.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to launch profiles YAML',
        ),
        DeclareLaunchArgument(
            'profile',
            default_value='vr_dual_arm_teleop',
            description='Profile name from launch_profiles.yaml',
        ),
        DeclareLaunchArgument('ros_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('ros_tcp_port', default_value='10000'),
        DeclareLaunchArgument('auto_detect_ip', default_value='true'),
        DeclareLaunchArgument('left_serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('right_serial_port', default_value='/dev/ttyACM1'),
        DeclareLaunchArgument('left_calibration_path', default_value=''),
        DeclareLaunchArgument('right_calibration_path', default_value=''),
        DeclareLaunchArgument('driver_backend', default_value='feetech'),
        DeclareLaunchArgument('auto_enable', default_value='false'),
        DeclareLaunchArgument('gripper_scale', default_value='2.0'),
        DeclareLaunchArgument('left_webui_port', default_value='8080'),
        DeclareLaunchArgument('right_webui_port', default_value='8081'),
        DeclareLaunchArgument('enable_foxglove', default_value='true'),
        DeclareLaunchArgument('enable_bridge', default_value='true'),
        DeclareLaunchArgument('fluent_vision_config', default_value='/config/fluent_vision_system.yaml'),
        DeclareLaunchArgument('fluent_vision_update_serials', default_value='false'),
        DeclareLaunchArgument('fluent_vision_serials_path', default_value='/config/camera_serials.yaml'),
        DeclareLaunchArgument('fluent_vision_use_serials_script', default_value='false'),
        DeclareLaunchArgument('fluent_vision_serials_script_path', default_value=''),
        DeclareLaunchArgument('rviz_config', default_value=''),
        DeclareLaunchArgument('rviz_vr_config', default_value=''),
        DeclareLaunchArgument('gazebo_args', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
