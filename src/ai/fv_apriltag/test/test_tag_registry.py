from pathlib import Path

from fv_apriltag.tag_registry import (
    parse_tag_size_overrides,
    TagRegistryError,
    TagSizeRegistry,
    TagSizeResolver,
)
import pytest


REGISTRY_PATH = Path(__file__).parents[1] / 'config' / 'tag_registry.yaml'


def test_repository_registry_uses_black_edge_sizes():
    registry = TagSizeRegistry.from_file(REGISTRY_PATH)

    assert registry.family == 'tag36h11'
    assert registry.size_for(0) == pytest.approx(0.040)
    assert registry.size_for(300) == pytest.approx(0.040)

    for id_range in (range(20, 53), range(110, 120)):
        for tag_id in id_range:
            assert registry.size_for(tag_id) == pytest.approx(0.120)


def test_default_resolver_uses_active_registry_mode():
    registry = TagSizeRegistry.from_file(REGISTRY_PATH)
    resolver = TagSizeResolver.create(registry)

    assert resolver.mode_description() == 'registry'
    assert resolver.size_for(0) == pytest.approx(0.040)
    assert resolver.size_for(20) == pytest.approx(0.120)


def test_reserved_unknown_and_legacy_ids_have_no_active_size():
    registry = TagSizeRegistry.from_file(REGISTRY_PATH)

    assert registry.size_for(53) is None
    assert registry.size_for(60) is None
    assert registry.size_for(120) is None
    assert registry.size_for(999) is None


def test_explicit_single_size_applies_to_every_detected_id():
    registry = TagSizeRegistry.from_file(REGISTRY_PATH)
    resolver = TagSizeResolver.create(registry, single_size_m=0.037)

    assert resolver.size_for(0) == pytest.approx(0.037)
    assert resolver.size_for(53) == pytest.approx(0.037)
    assert resolver.size_for(999) == pytest.approx(0.037)
    assert resolver.reference_size() == pytest.approx(0.037)
    assert resolver.mode_description() == 'single_override(0.037000m)'


def test_per_id_size_overrides_registry_and_can_enable_a_reserved_id():
    registry = TagSizeRegistry.from_file(REGISTRY_PATH)
    resolver = TagSizeResolver.create(
        registry,
        per_id_raw=[0.0, 0.041, 53.0, 0.055],
    )

    assert resolver.size_for(0) == pytest.approx(0.041)
    assert resolver.size_for(20) == pytest.approx(0.120)
    assert resolver.size_for(53) == pytest.approx(0.055)
    assert resolver.size_for(54) is None


def test_single_and_per_id_overrides_are_rejected_together():
    registry = TagSizeRegistry.from_file(REGISTRY_PATH)

    with pytest.raises(TagRegistryError, match='cannot be configured together'):
        TagSizeResolver.create(
            registry,
            single_size_m=0.04,
            per_id_raw=[0.0, 0.04],
        )


@pytest.mark.parametrize(
    'raw',
    (
        [0.0],
        [-1.0, 0.04],
        [0.5, 0.04],
        [0.0, 0.0],
        [0.0, -0.04],
        [0.0, 0.04, 0.0, 0.05],
    ),
)
def test_invalid_per_id_overrides_fail_closed(raw):
    with pytest.raises(TagRegistryError):
        parse_tag_size_overrides(raw)


def test_overlapping_active_allocations_are_rejected():
    raw = {
        'family': 'tag36h11',
        'allocations': [
            {'ids': [0, 2], 'tag_mm': 50},
            {'ids': [2, 3], 'tag_mm': 50},
        ],
    }

    with pytest.raises(TagRegistryError, match='overlaps'):
        TagSizeRegistry.from_mapping(raw)


def test_legacy_allocations_are_not_loaded():
    raw = {
        'family': 'tag36h11',
        'allocations': [{'ids': [0, 0], 'tag_mm': 50}],
        'legacy': [{'ids': [1, 1], 'tag_mm': 150}],
    }

    registry = TagSizeRegistry.from_mapping(raw)

    assert registry.size_for(0) == pytest.approx(0.040)
    assert registry.size_for(1) is None
