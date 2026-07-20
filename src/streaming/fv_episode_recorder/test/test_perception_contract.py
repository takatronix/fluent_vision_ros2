from pathlib import Path

from fv_episode_recorder.annotation_store import AnnotationStore
from fv_episode_recorder.moss_rounds import EpisodeRoundOwnership, MossRoundParser


def test_post_ended_dynamic_corrections_update_without_republish(
    tmp_path: Path,
) -> None:
    store = AnnotationStore(tmp_path / "annotations.db")
    parser = MossRoundParser()
    ownership = EpisodeRoundOwnership()
    episode_id = "episode-e2e"
    published: list[str] = []

    store.start_episode(episode_id)
    ownership.start(episode_id, now=0.0)
    for completed in parser.push_routed(
        "<|round_start|><|response|>赤い箱が机に置かれた。<|round_end|>",
        episode_id=ownership.route_episode(now=0.5),
    ):
        assert completed.episode_id is not None
        assert completed.text is not None
        assert ownership.claim_completion(completed.episode_id, now=0.5)
        write = store.upsert_annotation(
            annotation_id=f"{completed.episode_id}:moss",
            episode_id=completed.episode_id,
            text=completed.text,
        )
        if write.created:
            published.append(write.text)

    store.end_episode(episode_id)
    assert ownership.end(episode_id, now=1.0)

    for completed in parser.push_routed(
        "<|round_start|><|response|>赤い工具箱が机の上に置かれた。<|round_end|>",
        episode_id=ownership.route_episode(now=1.1),
    ):
        if completed.episode_id is None or completed.text is None:
            continue
        if not ownership.claim_completion(completed.episode_id, now=1.2):
            continue
        write = store.upsert_annotation(
            annotation_id=f"{completed.episode_id}:moss",
            episode_id=completed.episode_id,
            text=completed.text,
        )
        if write.created:
            published.append(write.text)

    for completed in parser.push_routed(
        "<|round_start|><|response|>赤い工具箱が机の中央に置かれた。<|round_end|>",
        episode_id=ownership.route_episode(now=1.3),
    ):
        if (
            completed.episode_id is not None
            and completed.text is not None
            and ownership.claim_completion(completed.episode_id, now=1.3)
        ):
            store.upsert_annotation(
                annotation_id=f"{completed.episode_id}:moss",
                episode_id=completed.episode_id,
                text=completed.text,
            )

    assert published == ["赤い箱が机に置かれた。"]
    assert store.search("さっき赤い工具箱は机の中央に置かれましたか？", 5) == [
        "赤い工具箱が机の中央に置かれた。"
    ]
    store.close()
