#!/usr/bin/env python3
"""
FV Pipeline Editor — ROS2 Node
Visual node-based editor backend for FluentVision pipelines.

Responsibilities:
  - Scan and load node_manifest.yaml from all fv packages
  - Serve editor web UI (HTML/JS/CSS) via aiohttp
  - WebSocket API for pipeline CRUD, node launch/stop, preview streaming
  - Pipeline storage (independent YAML files in configurable directory)
  - Process management for launched ROS2 nodes
"""

import asyncio
import glob as globmod
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Optional: for camera preview
import numpy as np
try:
    import cv2
    from sensor_msgs.msg import Image, CompressedImage
    HAS_CV = True
except ImportError:
    HAS_CV = False

try:
    from aiohttp import web
except ImportError:
    print("[fv_pipeline_editor] ERROR: aiohttp not installed. "
          "Install with: pip3 install aiohttp", file=sys.stderr)
    sys.exit(1)


# ---------------------------------------------------------------------------
# Port compatibility table
# ---------------------------------------------------------------------------
FV_PORT_COMPAT: Dict[str, set] = {
    'image':       {'image', 'depth'},
    'depth':       {'depth'},
    'camera_info': {'camera_info'},
    'pointcloud':  {'pointcloud'},
    'detections':  {'detections'},
    'audio':       {'audio'},
    'any':         {'image', 'depth', 'camera_info', 'pointcloud',
                    'detections', 'audio', 'any'},
}


# ---------------------------------------------------------------------------
# Manifest loader
# ---------------------------------------------------------------------------
def find_manifest_files(search_roots: List[str]) -> List[str]:
    """Find all node_manifest.yaml files under search roots."""
    paths = []
    for root in search_roots:
        root = os.path.expanduser(root)
        pattern = os.path.join(root, '**', 'config', 'node_manifest.yaml')
        paths.extend(globmod.glob(pattern, recursive=True))
    return sorted(set(paths))


def load_node_manifests(search_roots: List[str]) -> Dict[str, Dict]:
    """Load all node_manifest.yaml files and return unified template dict."""
    templates: Dict[str, Dict] = {}
    manifest_files = find_manifest_files(search_roots)

    for path in manifest_files:
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            if not data or 'nodes' not in data:
                continue
            for node_def in data['nodes']:
                key = node_def.get('key')
                if not key:
                    continue
                templates[key] = {
                    'package': node_def.get('package', ''),
                    'exec': node_def.get('exec', ''),
                    'category': node_def.get('category', 'ai'),
                    'label': node_def.get('label', key),
                    'inputs': node_def.get('inputs', []),
                    'outputs': node_def.get('outputs', []),
                    'default_parameters': node_def.get('default_parameters', {}),
                    '_manifest_path': path,
                }
        except Exception as e:
            print(f"[manifest] Error loading {path}: {e}", file=sys.stderr)

    return templates


def find_container_manifest_files(search_roots: List[str]) -> List[str]:
    """Find all container_manifest.yaml files under search roots."""
    paths = []
    for root in search_roots:
        root = os.path.expanduser(root)
        pattern = os.path.join(root, '**', 'container_manifest.yaml')
        paths.extend(globmod.glob(pattern, recursive=True))
    return sorted(set(paths))


def load_container_manifests(search_roots: List[str]) -> Dict[str, Dict]:
    """Load all container_manifest.yaml files and return unified template dict."""
    templates: Dict[str, Dict] = {}
    manifest_files = find_container_manifest_files(search_roots)

    for path in manifest_files:
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            if not data or 'containers' not in data:
                continue
            for cont_def in data['containers']:
                key = cont_def.get('key')
                if not key:
                    continue
                templates[key] = {
                    'label': cont_def.get('label', key),
                    'category': cont_def.get('category', 'ai_runtime'),
                    'image': cont_def.get('image', ''),
                    'default_parameters': cont_def.get('default_parameters', {}),
                    '_manifest_path': path,
                }
        except Exception as e:
            print(f"[container_manifest] Error loading {path}: {e}", file=sys.stderr)

    return templates


# ---------------------------------------------------------------------------
# Parameter flattening for ros2 run -p
# ---------------------------------------------------------------------------
def _get_nested(d: Dict, dotted_key: str):
    """Lookup a dotted key like 'a.b.c' in a nested dict."""
    parts = dotted_key.split('.')
    obj = d
    for p in parts:
        if not isinstance(obj, dict) or p not in obj:
            return None
        obj = obj[p]
    return obj


def flatten_params(params: Dict[str, Any], prefix: str = '',
                   type_hints: Optional[Dict] = None) -> Dict[str, str]:
    """Flatten nested dict to 'key.subkey': 'value' for ros2 run -p.

    type_hints: template default_parameters used to preserve float types
    that get lost during JSON round-trip (JS converts 1.0 → 1).
    """
    result = {}
    for k, v in params.items():
        full_key = f'{prefix}.{k}' if prefix else k
        if isinstance(v, dict):
            result.update(flatten_params(v, full_key, type_hints))
        elif isinstance(v, bool):
            result[full_key] = 'true' if v else 'false'
        elif isinstance(v, list):
            result[full_key] = json.dumps(v)
        elif isinstance(v, int) and not isinstance(v, bool):
            # Check if this should be a float (lost in JSON round-trip)
            hint = _get_nested(type_hints, full_key) if type_hints else None
            if isinstance(hint, float):
                result[full_key] = str(float(v))
            else:
                result[full_key] = str(v)
        else:
            result[full_key] = str(v)
    return result


# ---------------------------------------------------------------------------
# Pipeline file I/O
# ---------------------------------------------------------------------------
def ensure_dir(path: str):
    """Create directory if it doesn't exist."""
    os.makedirs(os.path.expanduser(path), exist_ok=True)


def list_pipeline_files(directory: str) -> List[str]:
    """List pipeline YAML files in a directory (without extension)."""
    d = os.path.expanduser(directory)
    if not os.path.isdir(d):
        return []
    names = []
    for f in sorted(os.listdir(d)):
        if f.endswith('.yaml') or f.endswith('.yml'):
            names.append(os.path.splitext(f)[0])
    return names


def load_pipeline_file(directory: str, name: str) -> Optional[Dict]:
    """Load a single pipeline YAML file."""
    d = os.path.expanduser(directory)
    for ext in ('.yaml', '.yml'):
        path = os.path.join(d, name + ext)
        if os.path.isfile(path):
            with open(path, 'r') as f:
                return yaml.safe_load(f)
    return None


def save_pipeline_file(directory: str, name: str, data: Dict) -> bool:
    """Save pipeline data to a YAML file."""
    d = os.path.expanduser(directory)
    ensure_dir(d)
    path = os.path.join(d, name + '.yaml')
    try:
        with open(path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True,
                      sort_keys=False)
        return True
    except Exception as e:
        print(f"[pipeline] Error saving {path}: {e}", file=sys.stderr)
        return False


def delete_pipeline_file(directory: str, name: str) -> bool:
    """Delete a pipeline YAML file."""
    d = os.path.expanduser(directory)
    for ext in ('.yaml', '.yml'):
        path = os.path.join(d, name + ext)
        if os.path.isfile(path):
            os.remove(path)
            return True
    return False


# ---------------------------------------------------------------------------
# Main ROS2 Node
# ---------------------------------------------------------------------------
class FVPipelineEditorNode(Node):
    """Pipeline Editor backend node."""

    def __init__(self):
        super().__init__('fv_pipeline_editor')

        # Parameters
        self.declare_parameter('port', 8095)
        self.declare_parameter('pipeline_dir', '~/.fluent_vision/pipelines')
        self.declare_parameter('builtin_dir', '')
        self.declare_parameter('manifest_search_roots', [
            '/ros2_ws/src/fluent_vision_ros2/src',
        ])
        self.declare_parameter('container_manifest_search_roots', [
            '/ros2_ws/src/fluent_vision_ros2/docker',
        ])

        self.port = self.get_parameter('port').value
        self.pipeline_dir = self.get_parameter('pipeline_dir').value
        self.builtin_dir = self.get_parameter('builtin_dir').value
        self.manifest_roots = self.get_parameter('manifest_search_roots').value
        self.container_manifest_roots = self.get_parameter(
            'container_manifest_search_roots').value

        # Auto-detect builtin dir
        if not self.builtin_dir:
            # Look for pipelines/ next to src/
            for root in self.manifest_roots:
                candidate = os.path.join(os.path.dirname(root), 'pipelines')
                if os.path.isdir(candidate):
                    self.builtin_dir = candidate
                    break

        detected_container_roots = []
        for root in self.container_manifest_roots:
            expanded = os.path.expanduser(root)
            if os.path.isdir(expanded):
                detected_container_roots.append(expanded)
        if not detected_container_roots:
            for root in self.manifest_roots:
                candidate = os.path.join(os.path.dirname(root), 'docker')
                if os.path.isdir(candidate):
                    detected_container_roots.append(candidate)
        self.container_manifest_roots = sorted(set(detected_container_roots))

        # Load manifests
        self.templates = load_node_manifests(self.manifest_roots)
        self.container_templates = load_container_manifests(
            self.container_manifest_roots)
        self.get_logger().info(
            f'Loaded {len(self.templates)} node templates from manifests')
        for key in sorted(self.templates.keys()):
            t = self.templates[key]
            self.get_logger().info(
                f'  [{t["category"]}] {key}: {t["label"]} '
                f'(in={len(t["inputs"])}, out={len(t["outputs"])})')
        self.get_logger().info(
            f'Loaded {len(self.container_templates)} container templates')
        for key in sorted(self.container_templates.keys()):
            t = self.container_templates[key]
            self.get_logger().info(
                f'  [container:{t["category"]}] {key}: {t["label"]} '
                f'image={t["image"]}')

        # Process management
        self._processes: Dict[str, subprocess.Popen] = {}
        self._process_lock = threading.Lock()
        self._containers: Dict[str, Dict[str, str]] = {}
        self._container_lock = threading.Lock()

        # WebSocket clients
        self._ws_clients: Set[web.WebSocketResponse] = set()

        # Camera preview state (topic -> latest JPEG bytes)
        self._preview_cache: Dict[str, bytes] = {}
        self._preview_subs: Dict[str, Any] = {}

        # Ensure pipeline dir exists
        ensure_dir(self.pipeline_dir)

        self.get_logger().info(
            f'FV Pipeline Editor starting on port {self.port}')
        self.get_logger().info(f'  Pipeline dir: {self.pipeline_dir}')
        self.get_logger().info(f'  Builtin dir: {self.builtin_dir}')

    # -----------------------------------------------------------------------
    # Web server
    # -----------------------------------------------------------------------
    def create_web_app(self) -> web.Application:
        """Create aiohttp web application."""
        app = web.Application()

        # Find web directory
        web_dir = self._find_web_dir()
        self.get_logger().info(f'  Web dir: {web_dir}')

        # Routes
        app.router.add_get('/ws', self._ws_handler)
        app.router.add_get('/health', self._health_handler)

        # Static files (serve index.html at /)
        if web_dir and os.path.isdir(web_dir):
            app.router.add_get('/', self._index_handler)
            app.router.add_static('/static', web_dir, show_index=False)
            self._web_dir = web_dir
        else:
            self._web_dir = None
            self.get_logger().warn('Web directory not found!')

        return app

    def _find_web_dir(self) -> str:
        """Find the web/ directory (dev or installed)."""
        # Development: relative to this file
        dev_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'web')
        if os.path.isdir(dev_dir):
            return dev_dir

        # Installed: share directory
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('fv_pipeline_editor')
            installed_dir = os.path.join(share_dir, 'web')
            if os.path.isdir(installed_dir):
                return installed_dir
        except Exception:
            pass

        return dev_dir

    async def _index_handler(self, request):
        """Serve index.html."""
        index_path = os.path.join(self._web_dir, 'index.html')
        if os.path.isfile(index_path):
            return web.FileResponse(index_path)
        return web.Response(text='Pipeline Editor — index.html not found',
                            status=404)

    async def _health_handler(self, request):
        """Health check endpoint."""
        return web.json_response({
            'status': 'ok',
            'templates': len(self.templates),
            'container_templates': len(self.container_templates),
            'processes': len(self._processes),
            'containers': len(self._containers),
        })

    # -----------------------------------------------------------------------
    # WebSocket handler
    # -----------------------------------------------------------------------
    async def _ws_handler(self, request):
        """Main WebSocket handler."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self._ws_clients.add(ws)
        self.get_logger().info('WebSocket client connected')

        try:
            async for raw_msg in ws:
                if raw_msg.type == web.WSMsgType.TEXT:
                    try:
                        msg = json.loads(raw_msg.data)
                        await self._handle_ws_message(ws, msg)
                    except json.JSONDecodeError:
                        await ws.send_json({
                            'type': 'error', 'message': 'Invalid JSON'})
                elif raw_msg.type == web.WSMsgType.ERROR:
                    self.get_logger().warn(
                        f'WebSocket error: {ws.exception()}')
        finally:
            self._ws_clients.discard(ws)
            self.get_logger().info('WebSocket client disconnected')

        return ws

    async def _handle_ws_message(self, ws, msg: Dict):
        """Route incoming WebSocket messages."""
        msg_type = msg.get('type', '')

        handlers = {
            'get_templates': self._handle_get_templates,
            'list_pipelines': self._handle_list_pipelines,
            'load_pipeline': self._handle_load_pipeline,
            'save_pipeline': self._handle_save_pipeline,
            'delete_pipeline': self._handle_delete_pipeline,
            'set_pipeline_dir': self._handle_set_pipeline_dir,
            'launch_node': self._handle_launch_node,
            'stop_node': self._handle_stop_node,
            'launch_pipeline': self._handle_launch_pipeline,
            'stop_pipeline': self._handle_stop_pipeline,
            'get_statuses': self._handle_get_statuses,
            'launch_container': self._handle_launch_container,
            'stop_container': self._handle_stop_container,
            'get_container_statuses': self._handle_get_container_statuses,
            'get_container_logs': self._handle_get_container_logs,
            'subscribe_preview': self._handle_subscribe_preview,
            'unsubscribe_preview': self._handle_unsubscribe_preview,
        }

        handler = handlers.get(msg_type)
        if handler:
            await handler(ws, msg)
        else:
            await ws.send_json({
                'type': 'error',
                'message': f'Unknown message type: {msg_type}'
            })

    # -----------------------------------------------------------------------
    # Template API
    # -----------------------------------------------------------------------
    async def _handle_get_templates(self, ws, msg: Dict):
        """Send all node templates and port compatibility."""
        # Convert sets to lists for JSON serialization
        compat = {k: list(v) for k, v in FV_PORT_COMPAT.items()}
        await ws.send_json({
            'type': 'templates',
            'templates': self.templates,
            'container_templates': self.container_templates,
            'port_compat': compat,
        })

    # -----------------------------------------------------------------------
    # Pipeline CRUD API
    # -----------------------------------------------------------------------
    async def _handle_list_pipelines(self, ws, msg: Dict):
        """List pipelines in current directory + builtins."""
        pipeline_dir = msg.get('dir', self.pipeline_dir)
        user_pipelines = list_pipeline_files(pipeline_dir)
        builtin_pipelines = list_pipeline_files(self.builtin_dir) \
            if self.builtin_dir else []

        await ws.send_json({
            'type': 'pipeline_list',
            'dir': pipeline_dir,
            'pipelines': user_pipelines,
            'builtin': builtin_pipelines,
        })

    async def _handle_load_pipeline(self, ws, msg: Dict):
        """Load a specific pipeline file."""
        name = msg.get('name', '')
        is_builtin = msg.get('builtin', False)
        directory = self.builtin_dir if is_builtin else \
            msg.get('dir', self.pipeline_dir)

        data = load_pipeline_file(directory, name)
        if data is not None:
            await ws.send_json({
                'type': 'pipeline_data',
                'name': name,
                'builtin': is_builtin,
                'pipeline': data,
            })
        else:
            await ws.send_json({
                'type': 'error',
                'message': f'Pipeline not found: {name}'
            })

    async def _handle_save_pipeline(self, ws, msg: Dict):
        """Save a pipeline to file."""
        name = msg.get('name', '')
        pipeline = msg.get('pipeline', {})
        directory = msg.get('dir', self.pipeline_dir)

        if not name:
            await ws.send_json({
                'type': 'save_result', 'success': False,
                'message': 'Pipeline name is required'
            })
            return

        # Add timestamps
        now = time.strftime('%Y-%m-%dT%H:%M:%S')
        if 'created' not in pipeline:
            pipeline['created'] = now
        pipeline['modified'] = now

        success = save_pipeline_file(directory, name, pipeline)
        await ws.send_json({
            'type': 'save_result',
            'success': success,
            'name': name,
        })

    async def _handle_delete_pipeline(self, ws, msg: Dict):
        """Delete a user pipeline."""
        name = msg.get('name', '')
        directory = msg.get('dir', self.pipeline_dir)

        success = delete_pipeline_file(directory, name)
        await ws.send_json({
            'type': 'delete_result',
            'success': success,
            'name': name,
        })

    async def _handle_set_pipeline_dir(self, ws, msg: Dict):
        """Change the pipeline directory."""
        new_dir = msg.get('dir', '')
        if new_dir:
            self.pipeline_dir = new_dir
            ensure_dir(new_dir)
            self.get_logger().info(f'Pipeline dir changed to: {new_dir}')

        # Return updated list
        await self._handle_list_pipelines(ws, {'dir': self.pipeline_dir})

    # -----------------------------------------------------------------------
    # Container launch / stop
    # -----------------------------------------------------------------------
    @staticmethod
    def _sanitize_container_name(name: str) -> str:
        cleaned = re.sub(r'[^a-zA-Z0-9_.-]+', '-', name).strip('-')
        return cleaned or 'fv-container'

    @staticmethod
    def _expand_value(value: str) -> str:
        return os.path.expanduser(os.path.expandvars(str(value)))

    def _merge_container_params(self, template_key: str,
                                parameters: Dict[str, Any]) -> Dict[str, Any]:
        defaults = self.container_templates.get(template_key, {}).get(
            'default_parameters', {})
        merged = json.loads(json.dumps(defaults))
        if isinstance(parameters, dict):
            for key, value in parameters.items():
                merged[key] = value
        return merged

    def _build_docker_run_cmd(self, container_id: str,
                              template_key: str,
                              parameters: Dict[str, Any]) -> tuple[List[str], str]:
        merged = self._merge_container_params(template_key, parameters)
        image = str(merged.get('image') or self.container_templates.get(
            template_key, {}).get('image', '')).strip()
        if not image:
            raise ValueError(f'container template "{template_key}" has no image')

        container_name = self._sanitize_container_name(
            str(merged.get('container_name', container_id or template_key)))
        cmd = ['docker', 'run', '-d', '--rm', '--name', container_name]

        if bool(merged.get('network_host', True)):
            cmd += ['--net=host']

        if bool(merged.get('gpu', False)):
            cmd += ['--gpus', 'all']

        for env_file in merged.get('env_files', []) or []:
            env_file_path = self._expand_value(env_file)
            if env_file_path:
                cmd += ['--env-file', env_file_path]

        for env_key, env_value in (merged.get('env', {}) or {}).items():
            cmd += ['-e', f'{env_key}={self._expand_value(env_value)}']

        for volume in merged.get('volumes', []) or []:
            if not isinstance(volume, dict):
                continue
            host_path = self._expand_value(volume.get('host', ''))
            container_path = str(volume.get('container', '')).strip()
            if not host_path or not container_path:
                continue
            spec = f'{host_path}:{container_path}'
            if volume.get('read_only', False):
                spec += ':ro'
            cmd += ['-v', spec]

        for port_map in merged.get('ports', []) or []:
            if not isinstance(port_map, dict):
                continue
            host_port = str(port_map.get('host', '')).strip()
            container_port = str(port_map.get('container', '')).strip()
            if host_port and container_port:
                cmd += ['-p', f'{host_port}:{container_port}']

        cmd.append(image)

        command = merged.get('command', [])
        if isinstance(command, str) and command.strip():
            cmd.append(command.strip())
        elif isinstance(command, list):
            cmd.extend([str(x) for x in command if str(x).strip()])

        return cmd, container_name

    def _container_name_from_spec(self, container_id: str, template_key: str,
                                  parameters: Dict[str, Any]) -> str:
        """Resolve the Docker container name from a manifest-backed spec."""
        if template_key and template_key in self.container_templates:
            merged = self._merge_container_params(template_key, parameters)
            return self._sanitize_container_name(
                str(merged.get('container_name', container_id or template_key)))
        return self._sanitize_container_name(container_id)

    def _resolve_container_name(self, container_id: str, msg: Dict) -> str:
        with self._container_lock:
            info = self._containers.get(container_id)
        if info and info.get('name'):
            return info['name']
        return self._container_name_from_spec(
            container_id,
            msg.get('template_key', ''),
            msg.get('parameters', {}) or {},
        )

    def _inspect_container(self, container_name: str) -> Dict[str, str]:
        result = subprocess.run(
            [
                'docker', 'inspect', '--format',
                '{{.State.Status}}|{{if .State.Health}}{{.State.Health.Status}}{{end}}',
                container_name,
            ],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if result.returncode != 0:
            return {'status': 'missing', 'health': ''}
        parts = result.stdout.strip().split('|', 1)
        status = parts[0] if parts else 'unknown'
        health = parts[1] if len(parts) > 1 else ''
        return {'status': status, 'health': health}

    async def _broadcast_container_status(self, container_id: str, status: str,
                                          health: str = '',
                                          message: str = ''):
        payload = {
            'type': 'container_status',
            'container_id': container_id,
            'status': status,
        }
        if health:
            payload['health'] = health
        if message:
            payload['message'] = message
        msg = json.dumps(payload)
        dead_clients = set()
        for ws in self._ws_clients:
            try:
                await ws.send_str(msg)
            except Exception:
                dead_clients.add(ws)
        self._ws_clients -= dead_clients

    async def _handle_launch_container(self, ws, msg: Dict):
        container_id = msg.get('container_id', '')
        template_key = msg.get('template_key', '')
        parameters = msg.get('parameters', {})

        if not template_key:
            await ws.send_json({
                'type': 'container_launch_result',
                'success': False,
                'container_id': container_id,
                'message': 'template_key required',
            })
            return

        if template_key not in self.container_templates:
            await ws.send_json({
                'type': 'container_launch_result',
                'success': False,
                'container_id': container_id,
                'message': f'unknown container template: {template_key}',
            })
            return

        logical_id = container_id or template_key
        self._stop_container(logical_id)

        try:
            cmd, container_name = self._build_docker_run_cmd(
                logical_id, template_key, parameters)
            self.get_logger().info(
                f'Launching container [{logical_id}]: {" ".join(cmd[:8])}...')
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30,
            )
            if result.returncode != 0:
                raise RuntimeError(result.stderr.strip() or result.stdout.strip())

            with self._container_lock:
                self._containers[logical_id] = {
                    'name': container_name,
                    'template_key': template_key,
                    'image': self.container_templates[template_key].get(
                        'image', ''),
                }

            inspect = self._inspect_container(container_name)
            await ws.send_json({
                'type': 'container_launch_result',
                'success': True,
                'container_id': logical_id,
                'container_name': container_name,
                'status': inspect['status'],
                'health': inspect['health'],
            })
            await self._broadcast_container_status(
                logical_id, inspect['status'], inspect['health'])
        except Exception as e:
            self.get_logger().error(f'Container launch failed [{logical_id}]: {e}')
            await ws.send_json({
                'type': 'container_launch_result',
                'success': False,
                'container_id': logical_id,
                'message': str(e),
            })
            await self._broadcast_container_status(logical_id, 'error', message=str(e))

    def _stop_container(self, container_id: str) -> bool:
        with self._container_lock:
            info = self._containers.get(container_id)
        if not info:
            return False
        container_name = info.get('name', '')
        try:
            subprocess.run(
                ['docker', 'stop', container_name],
                capture_output=True,
                text=True,
                timeout=20,
            )
        finally:
            with self._container_lock:
                self._containers.pop(container_id, None)
        return True

    async def _handle_stop_container(self, ws, msg: Dict):
        container_id = msg.get('container_id', '')
        stopped = self._stop_container(container_id)
        if not stopped:
            container_name = self._resolve_container_name(container_id, msg)
            result = subprocess.run(
                ['docker', 'stop', container_name],
                capture_output=True,
                text=True,
                timeout=20,
            )
            stopped = result.returncode == 0
        await ws.send_json({
            'type': 'container_stop_result',
            'success': stopped,
            'container_id': container_id,
        })
        if stopped:
            await self._broadcast_container_status(container_id, 'stopped')

    async def _handle_get_container_statuses(self, ws, msg: Dict):
        statuses = {}
        requested = msg.get('containers', []) or []
        for cont in requested:
            if not isinstance(cont, dict):
                continue
            container_id = cont.get('id') or cont.get('template', '')
            if not container_id:
                continue
            template_key = cont.get('template', '')
            parameters = cont.get('parameters', {}) or {}
            container_name = self._container_name_from_spec(
                container_id, template_key, parameters)
            inspect = self._inspect_container(container_name)
            tmpl = self.container_templates.get(template_key, {})
            image = str((parameters or {}).get('image') or tmpl.get('image', ''))
            statuses[container_id] = {
                'status': inspect['status'],
                'health': inspect['health'],
                'template_key': template_key,
                'container_name': container_name,
                'image': image,
            }
        with self._container_lock:
            items = dict(self._containers)
        for container_id, info in items.items():
            inspect = self._inspect_container(info.get('name', ''))
            statuses[container_id] = {
                'status': inspect['status'],
                'health': inspect['health'],
                'template_key': info.get('template_key', ''),
                'container_name': info.get('name', ''),
                'image': info.get('image', ''),
            }
        await ws.send_json({
            'type': 'container_statuses',
            'statuses': statuses,
        })

    async def _handle_get_container_logs(self, ws, msg: Dict):
        container_id = msg.get('container_id', '')
        container_name = self._resolve_container_name(container_id, msg)
        inspect = self._inspect_container(container_name)
        if inspect['status'] == 'missing':
            await ws.send_json({
                'type': 'container_logs',
                'container_id': container_id,
                'success': False,
                'message': f'container not found: {container_name}',
                'logs': '',
            })
            return
        result = subprocess.run(
            ['docker', 'logs', '--tail', '200', container_name],
            capture_output=True,
            text=True,
            timeout=10,
        )
        logs = (result.stdout or '') + (result.stderr or '')
        await ws.send_json({
            'type': 'container_logs',
            'container_id': container_id,
            'success': result.returncode == 0,
            'logs': logs[-8000:],
        })

    # -----------------------------------------------------------------------
    # Node launch / stop
    # -----------------------------------------------------------------------
    async def _handle_launch_node(self, ws, msg: Dict):
        """Launch a single ROS2 node."""
        node_id = msg.get('node_id', '')
        package = msg.get('package', '')
        executable = msg.get('exec', '')
        node_name = msg.get('node_name', '')
        namespace = msg.get('namespace', '')
        parameters = msg.get('parameters', {})

        if not package or not executable:
            await ws.send_json({
                'type': 'launch_result', 'success': False,
                'node_id': node_id, 'message': 'package and exec required'
            })
            return

        # Kill existing if running (tracked by this editor session)
        self._kill_process(node_id)

        # Kill orphaned processes from previous editor sessions
        self._kill_orphaned(executable)

        # Build ros2 run command
        cmd = ['ros2', 'run', package, executable, '--ros-args']

        if node_name:
            cmd += ['-r', f'__node:={node_name}']
        if namespace:
            cmd += ['-r', f'__ns:={namespace}']

        # Flatten and add parameters (use template defaults as type hints)
        template_key = msg.get('_template', '')
        type_hints = None
        if template_key and template_key in self.templates:
            type_hints = self.templates[template_key].get(
                'default_parameters', {})
        flat = flatten_params(parameters, type_hints=type_hints)
        for k, v in flat.items():
            if v != '':  # Skip empty parameters
                cmd += ['-p', f'{k}:={v}']

        self.get_logger().info(
            f'Launching [{node_id}]: {" ".join(cmd[:8])}...')

        try:
            # Use CycloneDDS to avoid FastRTPS symbol collision with librealsense2
            launch_env = os.environ.copy()
            launch_env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,  # New process group for cleanup
                env=launch_env,
            )
            with self._process_lock:
                self._processes[node_id] = proc

            await ws.send_json({
                'type': 'launch_result',
                'success': True,
                'node_id': node_id,
                'pid': proc.pid,
            })

            # Broadcast status update
            await self._broadcast_status(node_id, 'running')

        except Exception as e:
            self.get_logger().error(f'Launch failed [{node_id}]: {e}')
            await ws.send_json({
                'type': 'launch_result',
                'success': False,
                'node_id': node_id,
                'message': str(e),
            })
            await self._broadcast_status(node_id, 'error')

    async def _handle_stop_node(self, ws, msg: Dict):
        """Stop a single node."""
        node_id = msg.get('node_id', '')
        killed = self._kill_process(node_id)

        await ws.send_json({
            'type': 'stop_result',
            'success': killed,
            'node_id': node_id,
        })

        if killed:
            await self._broadcast_status(node_id, 'stopped')

    async def _handle_launch_pipeline(self, ws, msg: Dict):
        """Launch all nodes in a pipeline with appropriate delays."""
        pipeline = msg.get('pipeline', {})
        system_cfg = pipeline.get('system', {})
        camera_delay = float(system_cfg.get('camera_start_delay', 2.0))
        default_delay = float(system_cfg.get('default_start_delay', 0.5))
        container_delay = float(system_cfg.get('container_start_delay', 1.0))
        nodes = pipeline.get('nodes', [])
        containers = pipeline.get('containers', [])

        launched = []
        launched_containers = []

        for cont in containers:
            if not cont.get('enable', True):
                continue
            await self._handle_launch_container(ws, {
                'container_id': cont.get('id', ''),
                'template_key': cont.get('template', ''),
                'parameters': cont.get('parameters', {}),
            })
            launched_containers.append(cont.get('id', cont.get('template', '')))
            if container_delay > 0:
                await asyncio.sleep(container_delay)

        # Separate camera (sensor) nodes and others
        sensor_nodes = []
        other_nodes = []

        for node_data in nodes:
            if not node_data.get('enable', True):
                continue
            # Determine category from template
            template_key = node_data.get('_template', '')
            tmpl = self.templates.get(template_key, {})
            category = tmpl.get('category', 'ai')

            if category == 'sensor':
                sensor_nodes.append(node_data)
            else:
                other_nodes.append(node_data)

        # Launch sensors first
        for nd in sensor_nodes:
            await self._handle_launch_node(ws, {
                'node_id': nd.get('id', ''),
                'package': nd.get('package', ''),
                'exec': nd.get('exec', ''),
                'node_name': nd.get('node_name', ''),
                'namespace': nd.get('namespace', ''),
                'parameters': nd.get('parameters', {}),
                '_template': nd.get('_template', ''),
            })
            launched.append(nd.get('id', ''))
            if camera_delay > 0:
                await asyncio.sleep(camera_delay)

        # Launch remaining nodes
        for nd in other_nodes:
            await self._handle_launch_node(ws, {
                'node_id': nd.get('id', ''),
                'package': nd.get('package', ''),
                'exec': nd.get('exec', ''),
                'node_name': nd.get('node_name', ''),
                'namespace': nd.get('namespace', ''),
                'parameters': nd.get('parameters', {}),
                '_template': nd.get('_template', ''),
            })
            launched.append(nd.get('id', ''))
            if default_delay > 0:
                await asyncio.sleep(default_delay)

        await ws.send_json({
            'type': 'launch_pipeline_result',
            'success': True,
            'launched': launched,
            'launched_containers': launched_containers,
        })

    async def _handle_stop_pipeline(self, ws, msg: Dict):
        """Stop all managed nodes."""
        stopped = []
        pipeline = msg.get('pipeline', {}) or {}
        requested_containers = pipeline.get('containers', []) or []
        with self._process_lock:
            node_ids = list(self._processes.keys())
        with self._container_lock:
            container_ids = list(self._containers.keys())

        for node_id in node_ids:
            self._kill_process(node_id)
            stopped.append(node_id)
            await self._broadcast_status(node_id, 'stopped')

        stopped_containers = []
        for container_id in container_ids:
            self._stop_container(container_id)
            stopped_containers.append(container_id)
            await self._broadcast_container_status(container_id, 'stopped')

        for cont in requested_containers:
            if not isinstance(cont, dict):
                continue
            container_id = cont.get('id', '')
            if not container_id or container_id in stopped_containers:
                continue
            container_name = self._container_name_from_spec(
                container_id,
                cont.get('template', ''),
                cont.get('parameters', {}) or {},
            )
            result = subprocess.run(
                ['docker', 'stop', container_name],
                capture_output=True,
                text=True,
                timeout=20,
            )
            if result.returncode == 0:
                stopped_containers.append(container_id)
                await self._broadcast_container_status(container_id, 'stopped')

        await ws.send_json({
            'type': 'stop_pipeline_result',
            'success': True,
            'stopped': stopped,
            'stopped_containers': stopped_containers,
        })

    async def _handle_get_statuses(self, ws, msg: Dict):
        """Return current status of all managed processes."""
        statuses = {}
        with self._process_lock:
            for node_id, proc in list(self._processes.items()):
                poll = proc.poll()
                if poll is None:
                    statuses[node_id] = 'running'
                elif poll == 0:
                    statuses[node_id] = 'stopped'
                else:
                    statuses[node_id] = 'error'

        await ws.send_json({
            'type': 'statuses',
            'statuses': statuses,
        })
        await self._handle_get_container_statuses(ws, msg)

    # -----------------------------------------------------------------------
    # Preview streaming
    # -----------------------------------------------------------------------
    @staticmethod
    def _imgmsg_to_bgr(image_msg) -> Optional[np.ndarray]:
        """Convert sensor_msgs/Image to BGR numpy array without cv_bridge."""
        enc = image_msg.encoding.lower()
        h, w = image_msg.height, image_msg.width
        data = bytes(image_msg.data)

        if enc in ('bgr8', 'bgr8; charset=utf-8'):
            img = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
        elif enc in ('rgb8', 'rgb8; charset=utf-8'):
            img = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif enc in ('bgra8',):
            img = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 4)
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        elif enc in ('rgba8',):
            img = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 4)
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        elif enc in ('mono8', '8uc1'):
            img = np.frombuffer(data, dtype=np.uint8).reshape(h, w)
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        elif enc in ('16uc1', 'mono16'):
            raw = np.frombuffer(data, dtype=np.uint16).reshape(h, w)
            # Normalize depth to grayscale (white=near, black=far)
            valid = raw[raw > 0]
            if valid.size > 0:
                mn, mx = valid.min(), valid.max()
                normed = np.zeros_like(raw, dtype=np.uint8)
                if mx > mn:
                    normed = (255 - ((raw.astype(np.float32) - mn)
                              / (mx - mn) * 255)).clip(0, 255).astype(np.uint8)
                img = cv2.cvtColor(normed, cv2.COLOR_GRAY2BGR)
            else:
                img = np.zeros((h, w, 3), dtype=np.uint8)
        elif enc in ('32fc1',):
            raw = np.frombuffer(data, dtype=np.float32).reshape(h, w)
            valid = raw[np.isfinite(raw) & (raw > 0)]
            if valid.size > 0:
                mn, mx = valid.min(), valid.max()
                normed = np.zeros_like(raw, dtype=np.uint8)
                if mx > mn:
                    normed = (255 - ((raw - mn) / (mx - mn) * 255)).clip(
                        0, 255).astype(np.uint8)
                img = cv2.cvtColor(normed, cv2.COLOR_GRAY2BGR)
            else:
                img = np.zeros((h, w, 3), dtype=np.uint8)
        elif enc in ('32fc3',):
            raw = np.frombuffer(data, dtype=np.float32).reshape(h, w, 3)
            # Normals range [-1, 1] → BGR [0, 255]
            img = ((raw * 127.5) + 127.5).clip(0, 255).astype(np.uint8)
        elif enc in ('32fc4',):
            raw = np.frombuffer(data, dtype=np.float32).reshape(h, w, 4)
            # RGBD: take first 3 channels (BGR), scale [0,1] → [0,255]
            img = (raw[:, :, :3] * 255).clip(0, 255).astype(np.uint8)
        elif enc == 'yuv422':
            yuv = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 2)
            img = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUYV)
        else:
            # Try as 3-channel uint8 fallback
            try:
                img = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
            except ValueError:
                return None
        return img

    async def _handle_subscribe_preview(self, ws, msg: Dict):
        """Subscribe to a ROS2 image topic for preview."""
        topic = msg.get('topic', '')
        if not topic or not HAS_CV:
            self.get_logger().warn(
                f'Preview rejected: topic={topic!r} HAS_CV={HAS_CV}')
            return

        # Destroy old subscription if exists (allows re-subscribe after reconnect)
        if topic in self._preview_subs:
            try:
                self.destroy_subscription(self._preview_subs.pop(topic))
            except Exception:
                pass
            self._preview_cache.pop(topic, None)

        # Create subscription
        def callback(image_msg, t=topic):
            try:
                cv_image = FVPipelineEditorNode._imgmsg_to_bgr(image_msg)
                if cv_image is None:
                    return
                # Resize for preview
                h, w = cv_image.shape[:2]
                max_w = 320
                if w > max_w:
                    scale = max_w / w
                    cv_image = cv2.resize(
                        cv_image, (max_w, int(h * scale)))
                _, jpeg = cv2.imencode(
                    '.jpg', cv_image,
                    [cv2.IMWRITE_JPEG_QUALITY, 70])
                self._preview_cache[t] = jpeg.tobytes()
            except Exception as e:
                self.get_logger().warn(
                    f'Preview encode error [{t}]: {e}', once=True)

        # Use BEST_EFFORT to match camera publishers (avoid QoS incompatibility)
        preview_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        sub = self.create_subscription(
            Image, topic, callback, preview_qos)
        self._preview_subs[topic] = sub
        self.get_logger().info(f'Preview subscribed: {topic}')

    async def _handle_unsubscribe_preview(self, ws, msg: Dict):
        """Unsubscribe from a preview topic."""
        topic = msg.get('topic', '')
        if topic in self._preview_subs:
            self.destroy_subscription(self._preview_subs.pop(topic))
            self._preview_cache.pop(topic, None)
            self.get_logger().info(f'Preview unsubscribed: {topic}')

    # -----------------------------------------------------------------------
    # Process management helpers
    # -----------------------------------------------------------------------
    def _kill_process(self, node_id: str) -> bool:
        """Terminate a managed process."""
        with self._process_lock:
            proc = self._processes.pop(node_id, None)

        if proc is None:
            return False

        try:
            # Send SIGTERM to process group
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=2)
            except Exception:
                pass
        except Exception:
            pass

        self.get_logger().info(f'Stopped [{node_id}]')
        return True

    def _kill_orphaned(self, executable: str):
        """Kill orphaned processes from previous editor sessions by executable name."""
        # Get PIDs of our tracked processes to avoid killing them
        tracked_pids = set()
        with self._process_lock:
            for proc in self._processes.values():
                tracked_pids.add(proc.pid)

        try:
            result = subprocess.run(
                ['pgrep', '-f', executable],
                capture_output=True, text=True, timeout=3)
            if result.returncode != 0:
                return
            for line in result.stdout.strip().split('\n'):
                if not line.strip():
                    continue
                pid = int(line.strip())
                if pid in tracked_pids or pid == os.getpid():
                    continue
                try:
                    os.kill(pid, signal.SIGKILL)
                    self.get_logger().info(
                        f'Killed orphaned process: {executable} (PID {pid})')
                except ProcessLookupError:
                    pass
        except Exception:
            pass

    def cleanup_all_processes(self):
        """Stop all managed processes on shutdown."""
        with self._process_lock:
            node_ids = list(self._processes.keys())
        for node_id in node_ids:
            self._kill_process(node_id)
        with self._container_lock:
            container_ids = list(self._containers.keys())
        for container_id in container_ids:
            self._stop_container(container_id)

    async def _broadcast_status(self, node_id: str, status: str,
                               message: str = ''):
        """Broadcast node status to all WebSocket clients."""
        payload = {
            'type': 'node_status',
            'node_id': node_id,
            'status': status,
        }
        if message:
            payload['message'] = message
        msg = json.dumps(payload)
        dead_clients = set()
        for ws in self._ws_clients:
            try:
                await ws.send_str(msg)
            except Exception:
                dead_clients.add(ws)
        self._ws_clients -= dead_clients

    async def _broadcast_previews(self):
        """Periodically send preview images to clients."""
        while True:
            await asyncio.sleep(0.2)  # 5 FPS max
            if not self._preview_cache or not self._ws_clients:
                continue

            for topic, jpeg_bytes in list(self._preview_cache.items()):
                # Send as binary with topic prefix
                header = topic.encode('utf-8') + b'\x00'
                data = header + jpeg_bytes
                dead_clients = set()
                for ws in self._ws_clients:
                    try:
                        await ws.send_bytes(data)
                    except Exception:
                        dead_clients.add(ws)
                self._ws_clients -= dead_clients

    # -----------------------------------------------------------------------
    # Status monitor
    # -----------------------------------------------------------------------
    async def _monitor_processes(self):
        """Check for crashed processes and notify clients."""
        while True:
            await asyncio.sleep(2.0)
            with self._process_lock:
                items = list(self._processes.items())

            for node_id, proc in items:
                poll = proc.poll()
                if poll is not None:
                    # Process exited
                    status = 'stopped' if poll == 0 else 'error'
                    stderr_output = ''
                    if poll != 0:
                        try:
                            stderr_output = proc.stderr.read().decode(
                                'utf-8', errors='replace')[-500:]
                        except Exception:
                            pass
                        self.get_logger().warn(
                            f'Node [{node_id}] crashed (exit={poll}): '
                            f'{stderr_output}')
                    with self._process_lock:
                        self._processes.pop(node_id, None)
                    await self._broadcast_status(
                        node_id, status,
                        stderr_output.strip() if stderr_output else '')

            with self._container_lock:
                container_items = list(self._containers.items())

            for container_id, info in container_items:
                inspect = self._inspect_container(info.get('name', ''))
                status = inspect['status']
                health = inspect['health']
                if status == 'missing':
                    with self._container_lock:
                        self._containers.pop(container_id, None)
                    await self._broadcast_container_status(
                        container_id, 'stopped')
                else:
                    await self._broadcast_container_status(
                        container_id, status, health)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = FVPipelineEditorNode()

    # Create web app
    app = node.create_web_app()

    # Run aiohttp and ROS2 spinning together
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    # ROS2 spin in a background thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Start background tasks
    loop.create_task(node._broadcast_previews())
    loop.create_task(node._monitor_processes())

    # Run web server
    try:
        runner = web.AppRunner(app)
        loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, '0.0.0.0', node.port)
        loop.run_until_complete(site.start())
        node.get_logger().info(
            f'Pipeline Editor ready at http://0.0.0.0:{node.port}/')
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down...')
        node.cleanup_all_processes()
        loop.run_until_complete(runner.cleanup())
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
