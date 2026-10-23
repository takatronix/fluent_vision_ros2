#!/usr/bin/env python3
"""
fv_mcp — MCP server that lets an LLM rewire FluentVision at runtime.

Two tool families:

  Pipeline (ROS graph, via fv_pipeline_editor):
    list_node_types / list_pipelines / get_pipeline / validate_pipeline /
    save_pipeline / run_pipeline / stop_pipeline

  Scene (fluent_stage GPU screen, via fvsc + scene_web hot reload):
    scene_describe / scene_status / scene_validate / scene_apply

Design notes:
- The server is NOT a ROS node. Pipeline launch/stop is proxied over the
  editor's existing WebSocket API, so there is exactly one launch path.
- scene_apply never bypasses validation: fvsc validate must pass first, then
  the .fvs file is replaced atomically (tmp + rename) and scene_web performs
  its own full validate + frame-boundary swap. A broken scene can therefore
  never reach the screen — same guarantee a human editor gets.

Configuration (environment):
  FV_REPO_ROOT      repository root (default: auto-detect from this file / cwd)
  FV_EDITOR_URL     fv_pipeline_editor base URL   (default http://localhost:8095)
  FV_FVSC           path to the fluent_stage fvsc binary
                    (default <repo>/core/fluent_stage/build/fvsc)
  FV_SCENE_FILE     the live .fvs file served by scene_web
  FV_SCENE_WEB_URL  scene_web base URL            (default http://localhost:8791)
"""

import asyncio
import json
import os
import subprocess
import tempfile
import time
from pathlib import Path

import yaml
from mcp.server.fastmcp import FastMCP

mcp = FastMCP('fluent-vision')


# --------------------------------------------------------------------------
# paths & config
# --------------------------------------------------------------------------

def _repo_root() -> Path:
    env = os.environ.get('FV_REPO_ROOT')
    if env:
        return Path(env)
    for base in (Path(__file__).resolve(), Path.cwd()):
        for parent in [base, *base.parents]:
            if (parent / 'pipelines').is_dir() and (parent / 'src').is_dir():
                return parent
    raise RuntimeError('repository root not found — set FV_REPO_ROOT')


REPO = _repo_root()
EDITOR_URL = os.environ.get('FV_EDITOR_URL', 'http://localhost:8095')
FVSC = Path(os.environ.get('FV_FVSC', REPO / 'core/fluent_stage/build/fvsc'))
SCENE_FILE = os.environ.get('FV_SCENE_FILE', '')
SCENE_WEB_URL = os.environ.get('FV_SCENE_WEB_URL', 'http://localhost:8791')
USER_PIPELINE_DIR = Path.home() / '.fluent_vision' / 'pipelines'


# --------------------------------------------------------------------------
# pipeline tools
# --------------------------------------------------------------------------

@mcp.tool()
def list_node_types() -> str:
    """List every node type available for pipelines (from node_manifest.yaml
    files across the source tree): key, package, exec, category, label,
    inputs/outputs and default parameters. This is the palette an LLM can
    wire pipelines from."""
    nodes = []
    for manifest in sorted(REPO.glob('src/*/*/config/node_manifest.yaml')):
        try:
            data = yaml.safe_load(manifest.read_text()) or {}
        except yaml.YAMLError as e:
            nodes.append({'manifest': str(manifest), 'error': str(e)})
            continue
        for node in data.get('nodes', []):
            node['_manifest'] = str(manifest.relative_to(REPO))
            nodes.append(node)
    return json.dumps(nodes, ensure_ascii=False, indent=1)


@mcp.tool()
def list_pipelines() -> str:
    """List pipeline YAML files: built-ins shipped in the repo and the user's
    own (~/.fluent_vision/pipelines)."""
    out = {'builtin': [], 'user': []}
    for f in sorted((REPO / 'pipelines').glob('*.yaml')):
        try:
            head = yaml.safe_load(f.read_text()) or {}
            out['builtin'].append({'name': f.stem,
                                   'description': head.get('description', '')})
        except yaml.YAMLError:
            out['builtin'].append({'name': f.stem, 'description': '(parse error)'})
    if USER_PIPELINE_DIR.is_dir():
        for f in sorted(USER_PIPELINE_DIR.glob('*.yaml')):
            out['user'].append({'name': f.stem})
    return json.dumps(out, ensure_ascii=False, indent=1)


def _pipeline_path(name: str, builtin: bool) -> Path:
    base = (REPO / 'pipelines') if builtin else USER_PIPELINE_DIR
    path = (base / name).with_suffix('.yaml')
    if base not in path.resolve().parents:
        raise ValueError('invalid pipeline name')
    return path


@mcp.tool()
def get_pipeline(name: str, builtin: bool = True) -> str:
    """Return a pipeline's YAML text. Set builtin=False for a user pipeline."""
    return _pipeline_path(name, builtin).read_text()


def _validate_pipeline_dict(data) -> list[str]:
    problems = []
    if not isinstance(data, dict):
        return ['top level must be a mapping']
    nodes = data.get('nodes')
    if not isinstance(nodes, list) or not nodes:
        return ["'nodes' must be a non-empty list"]
    known = set()
    for manifest in REPO.glob('src/*/*/config/node_manifest.yaml'):
        try:
            for n in (yaml.safe_load(manifest.read_text()) or {}).get('nodes', []):
                known.add(n.get('package'))
        except yaml.YAMLError:
            pass
    seen_ids = set()
    for i, node in enumerate(nodes):
        where = f'nodes[{i}]'
        if not isinstance(node, dict):
            problems.append(f'{where}: must be a mapping')
            continue
        for key in ('id', 'package', 'exec'):
            if not node.get(key):
                problems.append(f"{where}: missing '{key}'")
        nid = node.get('id')
        if nid in seen_ids:
            problems.append(f"{where}: duplicate id '{nid}'")
        seen_ids.add(nid)
        pkg = node.get('package')
        if pkg and known and pkg not in known:
            problems.append(f"{where}: package '{pkg}' has no node_manifest "
                            f'(may still exist — warning only)')
    return problems


@mcp.tool()
def validate_pipeline(yaml_text: str) -> str:
    """Validate pipeline YAML (editor schema: flat 'nodes:' list with
    id/package/exec/parameters). Returns problems, or ok."""
    try:
        data = yaml.safe_load(yaml_text)
    except yaml.YAMLError as e:
        return json.dumps({'ok': False, 'problems': [f'YAML parse error: {e}']})
    problems = _validate_pipeline_dict(data)
    return json.dumps({'ok': not problems, 'problems': problems},
                      ensure_ascii=False)


@mcp.tool()
def save_pipeline(name: str, yaml_text: str) -> str:
    """Validate, then save a pipeline into the user pipeline directory
    (~/.fluent_vision/pipelines/<name>.yaml). Refuses invalid pipelines."""
    verdict = json.loads(validate_pipeline(yaml_text))
    if not verdict['ok']:
        return json.dumps({'saved': False, **verdict}, ensure_ascii=False)
    USER_PIPELINE_DIR.mkdir(parents=True, exist_ok=True)
    path = _pipeline_path(name, builtin=False)
    path.write_text(yaml_text)
    return json.dumps({'saved': True, 'path': str(path)})


async def _editor_ws_call(messages: list[dict], collect: int = 1,
                          timeout: float = 20.0) -> list[dict]:
    import aiohttp
    replies = []
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(
                EDITOR_URL.replace('http', 'ws', 1) + '/ws') as ws:
            for m in messages:
                await ws.send_json(m)
            end = time.monotonic() + timeout
            while len(replies) < collect and time.monotonic() < end:
                msg = await ws.receive(timeout=max(0.1, end - time.monotonic()))
                if msg.type == aiohttp.WSMsgType.TEXT:
                    replies.append(json.loads(msg.data))
                elif msg.type in (aiohttp.WSMsgType.CLOSED, aiohttp.WSMsgType.ERROR):
                    break
    return replies


@mcp.tool()
def run_pipeline(name: str, builtin: bool = True) -> str:
    """Launch a pipeline's nodes via the running fv_pipeline_editor
    (must be up at FV_EDITOR_URL). Equivalent to pressing Run in the UI."""
    pipeline = yaml.safe_load(_pipeline_path(name, builtin).read_text())
    problems = _validate_pipeline_dict(pipeline)
    hard = [p for p in problems if 'warning only' not in p]
    if hard:
        return json.dumps({'launched': False, 'problems': hard}, ensure_ascii=False)
    try:
        replies = asyncio.run(_editor_ws_call(
            [{'type': 'launch_pipeline', 'pipeline': pipeline}], collect=3))
    except OSError as e:
        return json.dumps({'launched': False,
                           'error': f'editor unreachable at {EDITOR_URL}: {e}'})
    return json.dumps({'launched': True, 'editor_replies': replies},
                      ensure_ascii=False)


@mcp.tool()
def stop_pipeline() -> str:
    """Stop all pipeline nodes the editor launched."""
    try:
        replies = asyncio.run(_editor_ws_call(
            [{'type': 'stop_pipeline'}], collect=2))
    except OSError as e:
        return json.dumps({'stopped': False,
                           'error': f'editor unreachable at {EDITOR_URL}: {e}'})
    return json.dumps({'stopped': True, 'editor_replies': replies},
                      ensure_ascii=False)


# --------------------------------------------------------------------------
# scene tools (fluent_stage)
# --------------------------------------------------------------------------

def _run_fvsc(*args: str, input_text: str | None = None) -> dict:
    if not FVSC.exists():
        return {'error': f'fvsc not found at {FVSC} — build fluent_stage or set FV_FVSC'}
    proc = subprocess.run([str(FVSC), *args], capture_output=True, text=True,
                          input=input_text, timeout=30)
    return {'exit_code': proc.returncode,
            'stdout': proc.stdout.strip(), 'stderr': proc.stderr.strip()}


@mcp.tool()
def scene_describe() -> str:
    """Self-description of the fluent_stage Scene runtime (fvsc describe
    --json): schema version, available content types, filters, attributes and
    their parameters. Read this BEFORE writing or editing a scene."""
    return json.dumps(_run_fvsc('describe', '--json'), ensure_ascii=False)


@mcp.tool()
def scene_validate(fvs_text: str) -> str:
    """Type-check a Scene document (.fvs YAML) without touching the live
    screen. Returns fvsc diagnostics; exit_code 0 means valid."""
    with tempfile.NamedTemporaryFile('w', suffix='.fvs', delete=False) as f:
        f.write(fvs_text)
        tmp = f.name
    try:
        return json.dumps(_run_fvsc('validate', tmp), ensure_ascii=False)
    finally:
        os.unlink(tmp)


@mcp.tool()
def scene_status() -> str:
    """Live scene_web status: current digest, reload count, last errors and
    lint warnings."""
    import urllib.request
    try:
        with urllib.request.urlopen(SCENE_WEB_URL + '/status', timeout=5) as r:
            return r.read().decode()
    except OSError as e:
        return json.dumps({'error': f'scene_web unreachable at {SCENE_WEB_URL}: {e}'})


@mcp.tool()
def scene_apply(fvs_text: str) -> str:
    """Apply a Scene document to the live screen. Validates with fvsc first,
    then atomically replaces the .fvs file watched by scene_web, which
    re-validates and swaps at a frame boundary. Returns the resulting
    scene_web status so you can confirm the reload took effect."""
    if not SCENE_FILE:
        return json.dumps({'applied': False,
                           'error': 'FV_SCENE_FILE not set — start scene_web '
                                    'and point FV_SCENE_FILE at its .fvs'})
    verdict = _run_fvsc_validate_text(fvs_text)
    if verdict.get('exit_code') != 0:
        return json.dumps({'applied': False, 'validation': verdict},
                          ensure_ascii=False)
    before = _reload_count()
    target = Path(SCENE_FILE)
    tmp = target.with_suffix('.fvs.tmp')
    tmp.write_text(fvs_text)
    os.replace(tmp, target)  # atomic: scene_web never sees a partial file
    # wait for scene_web to pick it up
    status = None
    for _ in range(30):
        time.sleep(0.2)
        status = _status_dict()
        if status and status.get('reloads', -1) != before:
            break
    return json.dumps({'applied': True, 'validation': verdict,
                       'scene_web': status}, ensure_ascii=False)


def _run_fvsc_validate_text(fvs_text: str) -> dict:
    with tempfile.NamedTemporaryFile('w', suffix='.fvs', delete=False) as f:
        f.write(fvs_text)
        tmp = f.name
    try:
        return _run_fvsc('validate', tmp)
    finally:
        os.unlink(tmp)


def _status_dict() -> dict | None:
    try:
        return json.loads(scene_status())
    except (json.JSONDecodeError, TypeError):
        return None


def _reload_count() -> int:
    status = _status_dict()
    return status.get('reloads', -1) if status else -1


def main():
    mcp.run()


if __name__ == '__main__':
    main()
