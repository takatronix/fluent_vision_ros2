from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
import os
import subprocess
from typing import Dict, List, Any
import shutil


def load_config(config_path: str):
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def _flatten_params(d: Dict[str, Any], prefix: str = "") -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    for k, v in (d or {}).items():
        key = f"{prefix}.{k}" if prefix else str(k)
        if isinstance(v, dict):
            out.update(_flatten_params(v, key))
        else:
            out[key] = v
    return out


def _load_params_file_as_flat(params_file: str) -> Dict[str, Any]:
    flat_params: Dict[str, Any] = {}
    with open(params_file, 'r') as pf:
        raw = yaml.safe_load(pf) or {}
    if isinstance(raw, dict) and raw:
        root_key = next(iter(raw.keys()))
        if isinstance(raw[root_key], dict):
            ros_params = raw[root_key].get('ros__parameters', {})
            if isinstance(ros_params, dict):
                flat_params = _flatten_params(ros_params)
    return flat_params


def load_camera_serials(config_path: str) -> Dict[str, str]:
    # Prefer /config/camera_serials.yaml, else sibling to main config
    candidates = [
        '/config/camera_serials.yaml',
        os.path.join(os.path.dirname(config_path), 'camera_serials.yaml'),
    ]
    for p in candidates:
        if os.path.isfile(p):
            try:
                with open(p, 'r') as f:
                    data = yaml.safe_load(f) or {}
                # Normalize keys
                return {str(k).upper(): str(v) for k, v in data.items()}
            except Exception:
                return {}
    return {}


def enumerate_realsense_serials() -> List[str]:
    try:
        import pyrealsense2 as rs  # type: ignore
        ctx = rs.context()
        devs = ctx.query_devices()
        serials = []
        for i in range(devs.size()):
            d = devs[i]
            try:
                serials.append(d.get_info(rs.camera_info.serial_number))
            except Exception:
                pass
        return serials
    except Exception:
        return []


def detect_realsense_models() -> Dict[str, str]:
    """Return mapping like { 'D405': 'SERIAL', 'D415': 'SERIAL', 'D435': 'SERIAL' }.
    If multiple same models exist, keep the first detected serial.
    """
    out: Dict[str, str] = {}
    try:
        import pyrealsense2 as rs  # type: ignore
        ctx = rs.context()
        devs = ctx.query_devices()
        for i in range(devs.size()):
            d = devs[i]
            try:
                name = d.get_info(rs.camera_info.name)
                serial = d.get_info(rs.camera_info.serial_number)
            except Exception:
                continue
            upname = (name or '').upper()
            model = None
            if 'D405' in upname:
                model = 'D405'
            elif 'D415' in upname:
                model = 'D415'
            elif 'D435' in upname:
                model = 'D435'
            if model and model not in out and serial:
                out[model] = serial
    except Exception:
        return out
    return out


def get_openvino_available_devices() -> List[str]:
    try:
        # OpenVINO 2022.3+ API
        from openvino.runtime import Core  # type: ignore
        core = Core()
        return list(core.available_devices)
    except Exception:
        try:
            # Older API fallback
            import openvino as ov  # type: ignore
            core = ov.Core()
            return list(core.available_devices)
        except Exception:
            return []


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config').perform(context)
    update_serials = LaunchConfiguration('update_camera_serials').perform(context).lower() in ('1', 'true', 'yes', 'on')
    camera_serials_path = LaunchConfiguration('camera_serials_path').perform(context)
    use_serials_script = LaunchConfiguration('use_serials_script').perform(context).lower() in ('1', 'true', 'yes', 'on')
    serials_script_path = LaunchConfiguration('serials_script_path').perform(context)
    cfg = load_config(config_path)
    # Allow config file to enable auto-update as well
    cfg_update = bool(cfg.get('system', {}).get('update_camera_serials', False))
    update_serials = update_serials or cfg_update

    # Support both keys: 'groups' (preferred) and legacy 'pipelines'
    groups = cfg.get('groups') if cfg.get('groups') is not None else cfg.get('pipelines', [])

    # actions collector
    actions = []

    # Optionally detect and update camera serials file BEFORE reading it
    if update_serials:
        model_map = {}
        # Prefer external script, if provided
        if use_serials_script and serials_script_path and os.path.isfile(serials_script_path):
            try:
                # Run without update flag: it will write /tmp/camera_serials.txt
                subprocess.run([serials_script_path], check=False)
                tmp_path = '/tmp/camera_serials.txt'
                if os.path.isfile(tmp_path):
                    try:
                        with open(tmp_path, 'r') as f:
                            for line in f:
                                line = line.strip()
                                if not line or ':' not in line:
                                    continue
                                k, v = line.split(':', 1)
                                k = k.strip().upper()
                                v = v.strip()
                                if k and v and k not in model_map:
                                    model_map[k] = v
                        actions.append(LogInfo(msg=f"[fluent_vision_system] Parsed serials from script: {model_map}"))
                    except Exception as e:
                        actions.append(LogInfo(msg=f"[fluent_vision_system] Failed reading /tmp/camera_serials.txt: {e}"))
                else:
                    actions.append(LogInfo(msg=f"[fluent_vision_system] Serial script did not produce {tmp_path}"))
            except Exception as e:
                actions.append(LogInfo(msg=f"[fluent_vision_system] Failed to run serials script '{serials_script_path}': {e}"))

        # Fallback to internal detection if script not used or failed
        if not model_map:
            model_map = detect_realsense_models()

        if model_map:
            try:
                os.makedirs(os.path.dirname(camera_serials_path), exist_ok=True)
                with open(camera_serials_path, 'w') as f:
                    yaml.safe_dump(model_map, f, default_flow_style=False, sort_keys=True)
                actions.append(LogInfo(msg=f"[fluent_vision_system] Updated camera serials at {camera_serials_path}: {model_map}"))
            except Exception as e:
                actions.append(LogInfo(msg=f"[fluent_vision_system] Failed to write camera serials to {camera_serials_path}: {e}"))
        else:
            actions.append(LogInfo(msg="[fluent_vision_system] No RealSense devices detected to update camera serials"))

    # foxglove_bridgeの起動
    # - Config: system.enable_foxglove / system.foxglove.port
    # - Env override:
    #     FV_ENABLE_FOXGLOVE=1|0
    #     FV_FOXGLOVE_PORT=8765
    enable_fox = bool(cfg.get('system', {}).get('enable_foxglove', False))
    env_enable = os.environ.get('FV_ENABLE_FOXGLOVE')
    if env_enable is not None:
        enable_fox = str(env_enable).strip().lower() in ('1', 'true', 'yes', 'on')

    if enable_fox:
        fox_cfg = cfg.get('system', {}).get('foxglove', {}) or {}
        fox_port = int(fox_cfg.get('port', 8765))
        env_port = os.environ.get('FV_FOXGLOVE_PORT')
        if env_port:
            try:
                fox_port = int(str(env_port).strip())
            except Exception:
                pass
        fox_params = [{'port': fox_port}]
        fox_log_level = str(fox_cfg.get('log_level', '')).strip()
        env_log_level = os.environ.get('FV_FOXGLOVE_LOG_LEVEL')
        if env_log_level:
            fox_log_level = str(env_log_level).strip()

        # Humble環境では type_description_interfaces が存在せず WARN が大量に出ることがあるため、
        # 必要なら log_level を設定して抑制できるようにする（例: error）。
        fox_args = []
        if fox_log_level:
            fox_args = ['--ros-args', '--log-level', f'foxglove_bridge:={fox_log_level}']
        foxglove_bridge = Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=fox_params,
            arguments=fox_args,
        )
        actions.append(foxglove_bridge)

    # Preprocess: determine desired RealSense serials from camera_serials.yaml (now possibly updated)
    serial_map = load_camera_serials(config_path)
    present_serials = enumerate_realsense_serials()
    ov_devices = [d.upper() for d in get_openvino_available_devices()]

    nodes_cfg = []
    for group in groups:
        if not group.get('enable', True):
            continue
        for n in group.get('nodes', []):
            if not n.get('enable', True):
                continue
            nodes_cfg.append(n)

    # 直列起動: カメラ系(fv_realsense/fv_camera)の間隔を大きく取り、同時起動を避ける
    camera_delay = float(cfg.get('system', {}).get('camera_start_delay', 3.0))
    default_delay = float(cfg.get('system', {}).get('default_start_delay', 0.5))
    inter_group_delay = float(cfg.get('system', {}).get('inter_group_delay', 1.0))

    delay = 0.0
    for group in groups:
        if not group.get('enable', True):
            continue

        group_nodes = [n for n in group.get('nodes', []) if n.get('enable', True)]
        for n in group_nodes:
            params_file = str(n.get('params_file', '') or '').strip()
            inline_params = n.get('parameters', {})
            if inline_params is None:
                inline_params = {}
            if not isinstance(inline_params, dict):
                actions.append(LogInfo(msg=f"[fluent_vision_system] Skip node '{n.get('id','?')}' due to invalid parameters type (dict required)"))
                continue
            use_params_file = bool(params_file and os.path.isfile(params_file))
            if params_file and not use_params_file:
                actions.append(LogInfo(msg=f"[fluent_vision_system] params_file not found for '{n.get('id','?')}': {params_file} (fallback to inline parameters)"))
            if (not use_params_file) and (not inline_params):
                actions.append(LogInfo(msg=f"[fluent_vision_system] Skip node '{n.get('id','?')}' due to missing params_file and empty inline parameters"))
                continue
            # Avoid using '/' as namespace default; empty string means root namespace
            ns = n.get('namespace', '')
            if ns == '/':
                ns = ''
            # Log what we are about to launch
            params_src = params_file if use_params_file else '(inline)'
            actions.append(LogInfo(msg=f"[fluent_vision_system] Launching {n.get('package')}:{n.get('exec')} name={n.get('node_name', n.get('id'))} ns='{ns}' params='{params_src}'"))

            # Optional runtime parameter overrides (flat dict for this node)
            overrides = {}
            if n.get('package') == 'fv_realsense':
                node_name = n.get('node_name', n.get('id', ''))
                want_serial = None
                lname = node_name.lower()
                if 'd405' in lname and 'D405' in serial_map:
                    want_serial = serial_map['D405']
                elif 'd415' in lname and 'D415' in serial_map:
                    want_serial = serial_map['D415']
                # Validate presence if we can detect devices
                if want_serial and present_serials and want_serial not in present_serials:
                    actions.append(LogInfo(msg=f"[fluent_vision_system] Warning: desired serial {want_serial} for {node_name} not detected. Connected: {present_serials}"))
                if want_serial:
                    # rclcpp側のdeclare_parameterはドット区切りのキーを想定するため、
                    # ネスト辞書ではなくフラットなキーで上書きする
                    overrides['camera_selection.serial_number'] = str(want_serial)
                    overrides['camera_selection.selection_method'] = 'serial'
                # シリアル未検出でもYAML側のcamera_selection（name等）で選択させるのでスキップしない

            # Fallbacks when GPU is unavailable
            if 'GPU' not in ov_devices:
                if n.get('package') == 'fv_instance_seg':
                    # fv_instance_seg expects flat key 'device'
                    overrides['device'] = 'CPU'
                if n.get('package') == 'fv_object_detector':
                    # fv_object_detector declares nested parameter 'model.device'
                    overrides['model.device'] = 'CPU'
                if n.get('package') == 'fv_object_mask_generator':
                    # UNet マスク生成は model.device を参照
                    overrides['model.device'] = 'CPU'

            # 共通: YAMLパラメータをロードしてフラット化し、overridesをマージして渡す
            # これにより --params-file の二重指定を完全に避ける
            flat_params: Dict[str, Any] = {}
            if use_params_file:
                try:
                    flat_params = _load_params_file_as_flat(params_file)
                except Exception as e:
                    actions.append(LogInfo(msg=f"[fluent_vision_system] Failed to read params file {params_file}: {e}"))
            if inline_params:
                flat_params.update(_flatten_params(inline_params))

            # overridesはフラットキー前提
            for ok, ov in (overrides or {}).items():
                flat_params[ok] = ov

            node_parameters = [flat_params] if flat_params else ([overrides] if overrides else [])

            node_name = n.get('node_name', n.get('id'))

            if n['package'] == 'fv_realsense':
                # RealSenseへも parameters を直接渡す（--ros-args の二重挿入を避ける）
                # YAMLの内容はflat_paramsにフラット化済み。CLIでの -p 指定は不要。
                node = Node(
                    package='fv_realsense',
                    executable='fv_realsense_node',
                    name=node_name,
                    namespace=ns,
                    parameters=node_parameters,
                    output='screen',
                )
            else:
                respawn = bool(n.get('respawn', False))
                respawn_delay = float(n.get('respawn_delay', 2.0))
                # Instance segmentation occasionally hangs inside GPU inference; allow respawn.
                if n.get('package') == 'fv_instance_seg':
                    respawn = True
                node = Node(
                    package=n['package'],
                    executable=n['exec'],
                    name=node_name,
                    namespace=ns,
                    # パラメータはフラット辞書として渡す
                    parameters=node_parameters,
                    output='screen',
                    respawn=respawn,
                    respawn_delay=respawn_delay,
                )

            # 個別にlaunch_delayが指定されていれば優先
            node_delay = n.get('launch_delay', None)
            if node_delay is None:
                # カメラ系は大きめの間隔、その他はデフォルト間隔
                if n['package'] in ('fv_realsense', 'fv_camera'):
                    inc = camera_delay
                else:
                    inc = default_delay
            else:
                inc = float(node_delay)

            actions.append(TimerAction(period=delay, actions=[node]))
            delay += inc

        # グループ間に待機を入れて電源負荷をさらに低減
        delay += inter_group_delay
    
    # --- Optional: RTAB-Map integration -------------------------------------
    try:
        rtab_cfg: Dict[str, Any] = (cfg.get('system', {}).get('rtabmap') or {})
        rtab_enabled = bool(rtab_cfg.get('enable', False))
    except Exception:
        rtab_cfg = {}
        rtab_enabled = False

    if rtab_enabled:
        database_path = str(rtab_cfg.get('database_path', '/recordings/rtabmap/rtabmap.db'))
        localization = bool(rtab_cfg.get('localization', False))
        use_viz = bool(rtab_cfg.get('rtabmapviz', False))
        rtab_delay = float(rtab_cfg.get('launch_delay', 2.0))
        mode = str(rtab_cfg.get('mode', 'rgbd_single')).lower()
        dual_depth = bool(rtab_cfg.get('dual_depth', False))

        # 共通: オドメトリ（カメラ1側を使用）
        cam1_rgb = str(rtab_cfg.get('cam1', {}).get('rgb', '/fv/d415/color/image_raw'))
        cam1_depth = str(rtab_cfg.get('cam1', {}).get('depth', '/fv/d415/depth/image_rect_raw'))
        cam1_info = str(rtab_cfg.get('cam1', {}).get('info', '/fv/d415/color/camera_info'))

        odom_params: Dict[str, Any] = {
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'approx_sync': True,
            'approx_sync_max_interval': 0.08,
            'wait_for_transform': 0.2,
            'Odom/Strategy': '1',             # Frame-to-Frame
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',
            'Vis/RefineIterations': '5',
        }
        rgbd_odometry = Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[odom_params],
            remappings=[
                ('rgb/image', cam1_rgb),
                ('depth/image', cam1_depth),
                ('rgb/camera_info', cam1_info),
            ],
        )

        # デュアル深度カメラ構成
        if mode in ('rgbd_dual', 'dual') or dual_depth:
            cam0_rgb = str(rtab_cfg.get('cam0', {}).get('rgb', '/fv/d405/color/image_raw'))
            cam0_depth = str(rtab_cfg.get('cam0', {}).get('depth', '/fv/d405/depth/image_rect_raw'))
            cam0_info = str(rtab_cfg.get('cam0', {}).get('info', '/fv/d405/color/camera_info'))

            # 2系統のRGB-D同期ノード
            rgbd_sync0 = Node(
                package='rtabmap_sync',
                executable='rgbd_sync',
                name='rgbd_sync0',
                output='screen',
                parameters=[{'approx_sync': True, 'queue_size': 30}],
                remappings=[
                    ('rgb/image', cam0_rgb),
                    ('depth/image', cam0_depth),
                    ('rgb/camera_info', cam0_info),
                    ('rgbd_image', 'rgbd_image0'),
                ],
            )
            rgbd_sync1 = Node(
                package='rtabmap_sync',
                executable='rgbd_sync',
                name='rgbd_sync1',
                output='screen',
                parameters=[{'approx_sync': True, 'queue_size': 30}],
                remappings=[
                    ('rgb/image', cam1_rgb),
                    ('depth/image', cam1_depth),
                    ('rgb/camera_info', cam1_info),
                    ('rgbd_image', 'rgbd_image1'),
                ],
            )

            rtab_params: Dict[str, Any] = {
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'subscribe_rgbd': True,
                'rgbd_cameras': 2,
                'subscribe_scan': False,
                'subscribe_odom': True,
                'queue_size': 30,
                'wait_for_transform': 0.2,
                'database_path': database_path,
                'Mem/IncrementalMemory': not localization,
            }
            rtabmap = Node(
                package='rtabmap_slam',
                executable='rtabmap',
                name='rtabmap',
                output='screen',
                parameters=[rtab_params],
                remappings=[
                    ('rgbd_image0', 'rgbd_image0'),
                    ('rgbd_image1', 'rgbd_image1'),
                    ('odom', '/odom'),
                ],
                arguments=['-d'],
            )

            actions.append(LogInfo(msg='[fluent_vision_system] RTABMap dual RGB-D enabled'))
            actions.append(TimerAction(period=delay + rtab_delay, actions=[rgbd_sync0]))
            actions.append(TimerAction(period=delay + rtab_delay, actions=[rgbd_sync1]))
            actions.append(TimerAction(period=delay + rtab_delay + 0.3, actions=[rgbd_odometry]))
            actions.append(TimerAction(period=delay + rtab_delay + 0.6, actions=[rtabmap]))

        else:
            # シングルカメラ（既定: D415）
            rtab_params: Dict[str, Any] = {
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': False,
                'subscribe_odom': True,
                'approx_sync': True,
                'queue_size': 30,
                'wait_for_transform': 0.2,
                'database_path': database_path,
                'Mem/IncrementalMemory': not localization,
            }
            rtabmap = Node(
                package='rtabmap_slam',
                executable='rtabmap',
                name='rtabmap',
                output='screen',
                parameters=[rtab_params],
                remappings=[
                    ('rgb/image', cam1_rgb),
                    ('depth/image', cam1_depth),
                    ('rgb/camera_info', cam1_info),
                    ('odom', '/odom'),
                ],
                arguments=['-d'],
            )

            actions.append(LogInfo(msg='[fluent_vision_system] RTABMap single RGB-D enabled'))
            actions.append(TimerAction(period=delay + rtab_delay, actions=[rgbd_odometry]))
            actions.append(TimerAction(period=delay + rtab_delay + 0.5, actions=[rtabmap]))

        # 可視化は必要時のみ（単一カメラのトピックに接続）
        if use_viz:
            viz_node = Node(
                package='rtabmap_viz',
                executable='rtabmap_viz',
                name='rtabmapviz',
                output='screen',
                parameters=[{
                    'subscribe_depth': True,
                    'subscribe_rgb': True,
                    'subscribe_scan': False,
                    'subscribe_odom_info': True,
                    'frame_id': 'base_link',
                    'odom_frame_id': 'odom',
                    'wait_for_transform': 0.2,
                }],
                remappings=[
                    ('rgb/image', cam1_rgb),
                    ('depth/image', cam1_depth),
                    ('rgb/camera_info', cam1_info),
                    ('odom', '/odom'),
                ],
            )
            actions.append(TimerAction(period=delay + rtab_delay + 1.0, actions=[viz_node]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            description='Absolute path to fluent_vision_system yaml',
            default_value='/config/fluent_vision_system.yaml',
        ),
        DeclareLaunchArgument(
            'update_camera_serials',
            description='If true, detect RealSense models and write camera serials file before launching',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'camera_serials_path',
            description='Path to write/read camera_serials.yaml',
            default_value='/config/camera_serials.yaml',
        ),
        DeclareLaunchArgument(
            'use_serials_script',
            description='Use external script to detect serials (preferred)',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'serials_script_path',
            description='Path to update_camera_serials.sh (or compatible) script',
            default_value='',
        ),
        OpaqueFunction(function=launch_setup),
    ])
