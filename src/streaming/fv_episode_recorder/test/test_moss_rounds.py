from fv_episode_recorder.moss_rounds import EpisodeRoundOwnership, MossRoundParser


def test_waits_for_complete_round_across_chunk_boundaries() -> None:
    parser = MossRoundParser()

    assert parser.push("<|round_start|><|response|>A person") == []
    assert parser.push(" entered the room.<|round_") == []
    assert parser.push("end|>") == ["A person entered the room."]


def test_silence_and_control_tokens_are_not_emitted() -> None:
    parser = MossRoundParser()

    assert parser.push("<|round_start|><|silence|><|round_end|>") == []
    assert parser.push(
        "<|round_start|><|response|>A box moved.<|round_end|>"
        "<|round_start|><|response|>It was a toolbox.<|round_end|>"
    ) == ["A box moved.", "It was a toolbox."]


def test_episode_end_closes_future_rounds_but_keeps_inflight_round() -> None:
    parser = MossRoundParser()

    assert parser.push_routed(
        "<|round_start|><|response|>A person",
        episode_id="episode-1",
    ) == []
    completed = parser.push_routed(
        " entered.<|round_end|>",
        episode_id=None,
    )
    assert len(completed) == 1
    assert completed[0].episode_id == "episode-1"
    assert completed[0].text == "A person entered."

    late = parser.push_routed(
        "<|round_start|><|response|>Late correction.<|round_end|>",
        episode_id=None,
    )
    assert len(late) == 1
    assert late[0].episode_id is None
    assert late[0].text == "Late correction."


def test_round_started_after_episode_end_is_unowned() -> None:
    parser = MossRoundParser()

    completed = parser.push_routed(
        "<|round_start|><|response|>Too late.<|round_end|>",
        episode_id=None,
    )
    assert completed[0].episode_id is None


def test_first_round_may_begin_just_after_episode_end() -> None:
    ownership = EpisodeRoundOwnership()
    parser = MossRoundParser()

    ownership.start("episode-1", now=0.0)
    assert ownership.end("episode-1", now=1.0) is True
    completed = parser.push_routed(
        "<|round_start|><|response|>A box moved.<|round_end|>",
        episode_id=ownership.route_episode(now=3600.0),
    )

    assert completed[0].episode_id == "episode-1"
    assert ownership.claim_completion("episode-1", now=3600.1) is True
    assert ownership.route_episode(now=3600.1) == "episode-1"
    assert ownership.claim_completion("episode-1", now=7200.0) is True


def test_finalization_ownership_survives_latency_until_next_episode() -> None:
    ownership = EpisodeRoundOwnership()

    ownership.start("episode-1", now=0.0)
    ownership.end("episode-1", now=1.0)
    assert ownership.route_episode(now=3600.0) == "episode-1"

    ownership.start("episode-2", now=4.0)
    assert ownership.route_episode(now=4.0) == "episode-2"
    assert ownership.claim_completion("episode-1", now=4.0) is False
