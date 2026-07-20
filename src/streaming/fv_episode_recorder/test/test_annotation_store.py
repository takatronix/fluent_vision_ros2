from pathlib import Path
import threading

from fv_episode_recorder.annotation_store import (
    AnnotationStore,
    build_fts_query,
    default_episode_root,
)


def test_default_episode_root_is_writable_user_home(
    tmp_path: Path,
    monkeypatch,
) -> None:
    monkeypatch.delenv("FV_EPISODE_OUTPUT_DIR", raising=False)
    monkeypatch.setenv("HOME", str(tmp_path))

    assert default_episode_root() == tmp_path / ".aspa" / "episodes"


def test_annotation_is_created_then_corrected_in_place(tmp_path: Path) -> None:
    store = AnnotationStore(tmp_path / "annotations.db")
    store.start_episode("episode-1", started_at=10.0)

    initial = store.upsert_annotation(
        annotation_id="episode-1:moss",
        episode_id="episode-1",
        text="赤い箱が机の上に置かれた。",
        updated_at=11.0,
    )
    correction = store.upsert_annotation(
        annotation_id="unused-replacement-id",
        episode_id="episode-1",
        text="赤い工具箱が机の上に置かれた。",
        updated_at=12.0,
    )

    assert initial.created is True
    assert initial.revision == 0
    assert correction.created is False
    assert correction.annotation_id == "episode-1:moss"
    assert correction.revision == 1
    assert store.search("工具箱", 5) == ["赤い工具箱が机の上に置かれた。"]
    assert store.search("赤い箱", 5) == []
    store.close()


def test_search_contains_only_semantic_annotations(tmp_path: Path) -> None:
    store = AnnotationStore(tmp_path / "annotations.db")
    store.start_episode("raw-only", started_at=10.0)
    store.end_episode("raw-only", ended_at=10.8)

    assert store.search("環境変動", 5) == []
    store.close()


def test_search_limit_is_bounded_and_short_terms_do_not_match(tmp_path: Path) -> None:
    store = AnnotationStore(tmp_path / "annotations.db")
    for index in range(25):
        episode_id = f"episode-{index}"
        store.upsert_annotation(
            annotation_id=f"{episode_id}:moss",
            episode_id=episode_id,
            text=f"作業台の前に人物が現れた {index}",
            updated_at=float(index),
        )

    assert store.search("人物", 5) == []
    results = store.search("人物が現れた", 100)
    assert len(results) == 20
    assert results[0].endswith("24")
    store.close()


def test_ordinary_japanese_question_uses_trigram_or_search(tmp_path: Path) -> None:
    store = AnnotationStore(tmp_path / "annotations.db")
    store.upsert_annotation(
        annotation_id="episode-question:moss",
        episode_id="episode-question",
        text="赤い工具箱が机の上に置かれた。",
    )

    query = "さっき赤い工具箱は机の上に置かれましたか？"
    assert " OR " in build_fts_query(query)
    assert store.search(query, 5) == ["赤い工具箱が机の上に置かれた。"]
    store.close()


def test_multiple_store_process_equivalents_write_one_wal_database(
    tmp_path: Path,
) -> None:
    database_path = tmp_path / "annotations.db"
    stores: list[AnnotationStore | None] = [None] * 4
    barrier = threading.Barrier(len(stores))
    errors: list[Exception] = []

    def write_many(index: int) -> None:
        episode_id = f"concurrent-{index}"
        try:
            store = AnnotationStore(database_path)
            stores[index] = store
            barrier.wait()
            store.start_episode(episode_id)
            for revision in range(12):
                store.upsert_annotation(
                    annotation_id=f"{episode_id}:moss",
                    episode_id=episode_id,
                    text=f"concurrency item {index} revision {revision}",
                )
            store.end_episode(episode_id)
        except Exception as exc:
            errors.append(exc)
            barrier.abort()

    threads = [
        threading.Thread(target=write_many, args=(index,), daemon=True)
        for index in range(len(stores))
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=15.0)

    assert all(not thread.is_alive() for thread in threads)
    assert errors == []
    checker = AnnotationStore(database_path)
    for index in range(len(stores)):
        annotation = checker.get_annotation(f"concurrent-{index}")
        assert annotation is not None
        assert annotation.revision == 11
        assert annotation.text.endswith("revision 11")
    checker.close()
    for store in stores:
        assert store is not None
        store.close()
