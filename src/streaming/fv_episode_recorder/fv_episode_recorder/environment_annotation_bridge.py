"""Environment-change annotations backed by Episode Recorder markers."""

from __future__ import annotations

import threading
import unicodedata
from typing import TYPE_CHECKING, Iterable

if TYPE_CHECKING:
    from fv_episode_msgs.msg import EnvironmentChange, EnvironmentEvent
    from rclpy.node import Node

from .episode_store import utc_now_iso


_PLACEHOLDER_DESCRIPTION = "Environment change"
_TAG = "environment_change"


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
        self._lock = threading.Lock()
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

    def _on_change(self, msg: "EnvironmentChange") -> None:
        change_id = msg.episode_id.strip()
        if not change_id:
            return
        if msg.state == "started":
            self._start_marker(change_id)
        elif msg.state == "ended":
            self._stop_marker(change_id)
        else:
            self._node.get_logger().warning(
                f"ignored invalid environment state: {msg.state}"
            )

    def _start_marker(self, change_id: str) -> None:
        with self._lock:
            if change_id in self._marker_ids:
                return
            active = self._store.active
            if active is None:
                self._node.get_logger().debug(
                    "environment change not persisted: no recording is active"
                )
                return
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

    def _stop_marker(self, change_id: str) -> None:
        with self._lock:
            marker_id = self._marker_ids.get(change_id)
        if marker_id is None:
            return
        if self._mm.stop(marker_id, outcome="success") is None:
            self._store.patch_finalized_marker(
                marker_id,
                {"stopped_at": utc_now_iso(), "outcome": "success"},
            )

    def _on_annotation(self, msg: "EnvironmentEvent") -> None:
        change_id = msg.episode_id.strip()
        text = msg.text.strip()
        if not change_id or not text:
            return
        with self._lock:
            marker_id = self._marker_ids.get(change_id)
        if marker_id is None:
            self._node.get_logger().debug(
                "environment annotation not persisted: no recording marker exists"
            )
            return
        if self._mm.patch(marker_id, task_description=text) is None:
            self._store.patch_finalized_marker(
                marker_id,
                {"task_description": text},
            )

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
