"""Environment-change annotations backed by Episode Recorder markers."""

from __future__ import annotations

import asyncio
import os
import unicodedata
from collections import deque
from pathlib import Path
from typing import TYPE_CHECKING, Awaitable, Callable, Iterable

if TYPE_CHECKING:
    from fv_episode_msgs.msg import EnvironmentChange, EnvironmentEvent
    from rclpy.node import Node

from .episode_store import utc_now_iso
from .schemas import StartEpisodeRequest, StopEpisodeRequest


_PLACEHOLDER_DESCRIPTION = "Environment change"
_TAG = "environment_change"
_AUTO_TAG = "auto_recorded"


def _normalize(text: str) -> str:
    return "".join(
        char for char in unicodedata.normalize("NFKC", text).casefold()
        if not char.isspace()
    )


def _bigrams(text: str) -> set[str]:
    normalized = _normalize(text)
    if len(normalized) < 2:
        return {normalized} if normalized else set()
    return {normalized[index:index + 2] for index in range(len(normalized) - 1)}


def _lexical_score(query: str, candidate: str) -> float:
    normalized_query = _normalize(query)
    normalized_candidate = _normalize(candidate)
    if not normalized_query or not normalized_candidate:
        return 0.0
    if (
        normalized_query in normalized_candidate
        or normalized_candidate in normalized_query
    ):
        return 1.0
    query_terms = _bigrams(normalized_query)
    candidate_terms = _bigrams(normalized_candidate)
    if not query_terms or not candidate_terms:
        return 0.0
    overlap = len(query_terms & candidate_terms)
    return overlap / min(len(query_terms), len(candidate_terms))


class EnvironmentAnnotationBridge:
    """Map anomaly lifecycles and MOSS text onto the active recording marker."""

    def __init__(
        self,
        node: "Node",
        store,
        marker_manager,
        *,
        change_topic: str = "/environment/change",
        annotation_topic: str = "/environment/annotation",
        search_service: str = "/episode/search",
        default_search_limit: int = 5,
        enabled: bool = True,
    ) -> None:
        self._node = node
        self._store = store
        self._mm = marker_manager
        self._default_search_limit = max(1, default_search_limit)
        self._marker_ids: dict[str, str] = {}
        self._auto_owned_episodes: set[str] = set()
        self._change_episode_ids: dict[str, str] = {}
        self._active_changes_by_episode: dict[str, set[str]] = {}
        self._started_changes: set[str] = set()
        self._ended_changes: set[str] = set()
        self._pending_annotations: dict[str, str] = {}
        self._loop: asyncio.AbstractEventLoop | None = None
        self._start_episode: Callable[[StartEpisodeRequest], Awaitable] | None = None
        self._stop_episode: Callable[[str, StopEpisodeRequest], Awaitable] | None = None
        self._profiles_dir: Path | None = None
        self._auto_profile_override = ""
        self._events: deque[tuple[str, ...]] = deque()
        self._drain_task: asyncio.Task | None = None
        self._subscriptions = []
        self._search_service = None

        if enabled:
            from fv_episode_msgs.msg import EnvironmentChange, EnvironmentEvent
            from fv_episode_msgs.srv import EpisodeSearch

            self._subscriptions = [
                node.create_subscription(
                    EnvironmentChange, change_topic, self._on_change, 10
                ),
                node.create_subscription(
                    EnvironmentEvent, annotation_topic, self._on_annotation, 10
                ),
            ]
            self._search_service = node.create_service(
                EpisodeSearch, search_service, self._on_search
            )
            node.get_logger().info(
                "environment annotation bridge: "
                f"changes={change_topic} annotations={annotation_topic} "
                f"search={search_service}"
            )

    def bind_lifecycle(
        self,
        loop: asyncio.AbstractEventLoop,
        *,
        start_episode: Callable[[StartEpisodeRequest], Awaitable],
        stop_episode: Callable[[str, StopEpisodeRequest], Awaitable],
        profiles_dir: Path,
        auto_profile_override: str = "",
    ) -> None:
        """Bind recorder lifecycle operations to the aiohttp main loop."""
        self._loop = loop
        self._start_episode = start_episode
        self._stop_episode = stop_episode
        self._profiles_dir = Path(profiles_dir)
        self._auto_profile_override = auto_profile_override.strip()

    def _on_change(self, msg: "EnvironmentChange") -> None:
        change_id = str(msg.episode_id).strip()
        if not change_id:
            return
        self._dispatch(("change", change_id, str(msg.state)))

    def _dispatch(self, event: tuple[str, ...]) -> None:
        loop = self._loop
        if loop is None:
            self._node.get_logger().warning(
                "environment event ignored before recorder lifecycle binding"
            )
            return
        loop.call_soon_threadsafe(self._enqueue_event, event)

    def _enqueue_event(self, event: tuple[str, ...]) -> None:
        self._events.append(event)
        if self._drain_task is None or self._drain_task.done():
            self._drain_task = self._loop.create_task(self._drain_events())

    async def _drain_events(self) -> None:
        while self._events:
            event = self._events.popleft()
            try:
                if event[0] == "change":
                    await self._process_change(event[1], event[2])
                else:
                    self._process_annotation(event[1], event[2])
            except Exception as exc:
                self._node.get_logger().warning(
                    f"environment event processing failed: {exc}"
                )

    async def wait_idle(self) -> None:
        """Wait until callback events already submitted to the loop are done."""
        await asyncio.sleep(0)
        while self._drain_task is not None and not self._drain_task.done():
            await asyncio.shield(self._drain_task)
            await asyncio.sleep(0)

    async def _process_change(self, change_id: str, state: str) -> None:
        if state == "started":
            if change_id in self._started_changes or change_id in self._ended_changes:
                return
            if await self._start_marker(change_id):
                self._started_changes.add(change_id)
        elif state == "ended":
            if change_id in self._ended_changes:
                return
            if await self._stop_marker(change_id):
                self._ended_changes.add(change_id)
        else:
            self._node.get_logger().warning(
                f"ignored invalid environment state: {state}"
            )

    async def _start_marker(self, change_id: str) -> bool:
        if change_id in self._marker_ids:
            return True
        active = self._store.active
        if active is None:
            profile = self._resolve_auto_profile()
            if profile is None:
                self._node.get_logger().warning(
                    "environment auto-record skipped: no valid profile"
                )
                return False
            if self._start_episode is None:
                self._node.get_logger().warning(
                    "environment auto-record skipped: lifecycle is unavailable"
                )
                return False
            try:
                started = await self._start_episode(StartEpisodeRequest(
                    task_description=_PLACEHOLDER_DESCRIPTION,
                    profile=profile,
                    tags=[_TAG, _AUTO_TAG],
                ))
            except Exception as exc:
                self._node.get_logger().warning(
                    f"environment auto-record start failed: {exc}"
                )
                return False
            episode_id = str(getattr(started, "episode_id", ""))
            active = self._store.active
            if (
                not episode_id
                or active is None
                or active.episode_id != episode_id
            ):
                self._node.get_logger().warning(
                    "environment auto-record start returned no active episode"
                )
                return False
            self._auto_owned_episodes.add(episode_id)

        marker = self._mm.start(
            active.episode_id,
            _PLACEHOLDER_DESCRIPTION,
            kind="subtask",
            tags=[_TAG],
            attributes=[
                {"key": "environment_change_id", "value": change_id}
            ],
        )
        self._marker_ids[change_id] = marker.marker_id
        self._change_episode_ids[change_id] = active.episode_id
        self._active_changes_by_episode.setdefault(
            active.episode_id, set()
        ).add(change_id)
        pending_text = self._pending_annotations.pop(change_id, None)
        if pending_text:
            self._patch_marker(marker.marker_id, pending_text)
        return True

    async def _stop_marker(self, change_id: str) -> bool:
        marker_id = self._marker_ids.get(change_id)
        if marker_id is None:
            self._pending_annotations.pop(change_id, None)
            return True
        if self._mm.stop(marker_id, outcome="success") is None:
            self._store.patch_finalized_marker(
                marker_id,
                {"stopped_at": utc_now_iso(), "outcome": "success"},
            )

        episode_id = self._change_episode_ids.get(change_id)
        if episode_id is None:
            return True
        active_changes = self._active_changes_by_episode.get(episode_id)
        if active_changes is not None:
            active_changes.discard(change_id)
            if active_changes:
                return True
            self._active_changes_by_episode.pop(episode_id, None)

        if episode_id not in self._auto_owned_episodes:
            return True
        active = self._store.active
        if (
            active is None
            or active.episode_id != episode_id
        ):
            self._auto_owned_episodes.discard(episode_id)
            return True
        if self._stop_episode is None:
            return False
        try:
            await self._stop_episode(
                episode_id, StopEpisodeRequest(outcome="success")
            )
        except Exception as exc:
            self._node.get_logger().warning(
                f"environment auto-record stop failed: {exc}"
            )
            active = self._store.active
            completed = active is None or active.episode_id != episode_id
            if completed:
                self._auto_owned_episodes.discard(episode_id)
            return completed
        self._auto_owned_episodes.discard(episode_id)
        return True

    def _on_annotation(self, msg: "EnvironmentEvent") -> None:
        change_id = str(msg.episode_id).strip()
        text = str(msg.text).strip()
        if not change_id or not text:
            return
        self._dispatch(("annotation", change_id, text))

    def _process_annotation(self, change_id: str, text: str) -> None:
        marker_id = self._marker_ids.get(change_id)
        if marker_id is None:
            if change_id in self._ended_changes:
                return
            self._pending_annotations[change_id] = text
            return
        self._patch_marker(marker_id, text)

    def _patch_marker(self, marker_id: str, text: str) -> None:
        if self._mm.patch(marker_id, task_description=text) is None:
            self._store.patch_finalized_marker(
                marker_id,
                {"task_description": text},
            )

    def _resolve_auto_profile(self) -> str | None:
        profiles_dir = self._profiles_dir
        if profiles_dir is None:
            return None
        candidates: list[str] = []
        if self._auto_profile_override:
            candidates.append(self._auto_profile_override)
        env_profile = os.environ.get("VLABOR_PROFILE", "").strip()
        if env_profile:
            candidates.append(env_profile)
        try:
            active_profile_path = Path.home() / ".vlabor" / "profiles" / ".active_profile"
            if active_profile_path.exists():
                file_profile = active_profile_path.read_text().strip()
                if file_profile:
                    candidates.append(file_profile)
        except OSError as exc:
            self._node.get_logger().warning(
                f"active profile read failed: {exc}"
            )
        for profile in dict.fromkeys(candidates):
            if any(
                (profiles_dir / f"{profile}{suffix}").is_file()
                for suffix in (".yaml", ".yml")
            ):
                return profile
        return None

    def _annotation_texts(self) -> Iterable[str]:
        seen_marker_ids: set[str] = set()
        active = self._store.active
        if active is not None:
            for marker in reversed(self._mm.list(active.episode_id)):
                if (
                    _TAG not in marker.tags
                    or marker.task_description == _PLACEHOLDER_DESCRIPTION
                ):
                    continue
                seen_marker_ids.add(marker.marker_id)
                yield marker.task_description

        for meta, _size in self._store.list_episodes(limit=1000):
            for marker in reversed(meta.markers):
                marker_id = str(marker.get("marker_id") or "")
                if (
                    marker_id in seen_marker_ids
                    or _TAG not in (marker.get("tags") or [])
                ):
                    continue
                text = str(marker.get("task_description") or "").strip()
                if text and text != _PLACEHOLDER_DESCRIPTION:
                    seen_marker_ids.add(marker_id)
                    yield text

    def search(self, query: str, limit: int | None = None) -> list[str]:
        result_limit = max(1, limit or self._default_search_limit)
        scored = [
            (_lexical_score(query, text), order, text)
            for order, text in enumerate(self._annotation_texts())
        ]
        matches = [item for item in scored if item[0] >= 0.25]
        matches.sort(key=lambda item: (-item[0], item[1]))
        results: list[str] = []
        for _score, _order, text in matches:
            if text not in results:
                results.append(text)
            if len(results) >= result_limit:
                break
        return results

    def _on_search(self, request, response):
        requested_limit = int(request.limit) if request.limit else None
        response.texts = self.search(request.query, requested_limit)
        return response
