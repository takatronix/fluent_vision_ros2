# Fluent Scene — declarative GPU visual runtime

English | [日本語](README.md)

**🌿 Demo page (live HUD recreation, measured numbers, real renders):**
<https://claude.ai/code/artifact/25d03136-2389-4b50-8cd9-eb50075f7970>

Fluent Scene executes a declarative `.fvs` scene (YAML) through
**parse → type validation → canonical IR → resource planning → retained Vulkan
rendering → ROS 2 wiring**. Roadmap stages **MVP + 1–4** of the design spec
([fluent_vision_architecture.md](../../docs/design/fluent_vision_architecture.md))
are implemented. The core links **no ROS 2, no learning framework, no GPU
libraries**; the renderer and the ROS 2 adapter are separate targets that
auto-disable when their dependencies are absent. A `COLCON_IGNORE` marker keeps
the whole tree out of colcon workspaces.

## Build & test

```bash
cd core/fluent_scene
cmake -B build -S . && cmake --build build -j
ctest --test-dir build          # contract + binding + render suites
```

The ROS 2 adapter additionally needs a sourced ROS environment:

```bash
source /opt/ros/<distro>/setup.bash
cmake -B build -S . -DCMAKE_PREFIX_PATH=/opt/ros/<distro> && cmake --build build -j
```

## Tools

| Tool | What it does |
|---|---|
| `fvsc parse\|validate\|compile` | bounded YAML parse, canonical IR + digest, resource/pass plan with budget rejection |
| `fvs_bench` | scene-compile latency (parse / validate / plan percentiles) |
| `fv_render` | headless retained rendering with synthetic inputs; `--backend both` measures GPU vs CPU; `--bindings` drives through the runtime binding table |
| `fv_scene_node` | ROS 2 adapter: binding-document topics → snapshot → render → publish |

## Measured on NVIDIA Thor (2026-08-13)

- Scene compile (structural edits only): ~153 µs for the spec §11 example.
- 1080p composite (camera + 5 boxes + Japanese text): **Vulkan 1.67 ms/frame
  (0.37 ms GPU execution) vs 43.2 ms scalar CPU — ~26×**; pixel outputs agree
  within a mean difference of <1/255.

## Status

Implemented: bounded parser (duplicate-key rejection, source spans), schema/type
validator with deterministic canonical IR + sha256 digest, backend-neutral
resource/pass planner with deterministic budget rejection, retained headless
Vulkan renderer + CPU reference behind one narrow interface (no per-frame
shader compilation, counter-proven), runtime binding table with frame-boundary
immutable snapshots (latest/fifo/approximate-sync policies, freshness, declared
fallbacks), `fluent.binding/v1alpha1` document validation, and a ROS 2 adapter
node verified end-to-end over DDS on an isolated domain.

Not yet: point clouds/3D/effects (stage 5), MCP lifecycle (stage 6), window
presentation, zero-copy import, depth rendering. See the Japanese
[README.md](README.md) for the full details and caveats.
