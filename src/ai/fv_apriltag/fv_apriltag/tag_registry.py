"""Pure tag-size registry loading and resolution for :mod:`fv_apriltag`."""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Dict, Mapping, Optional, Sequence

import yaml


BLACK_EDGE_RATIO = 0.8
MM_PER_M = 1000.0


class TagRegistryError(ValueError):
    """Raised when a registry or runtime size override is unsafe to use."""


@dataclass(frozen=True)
class TagSizeRegistry:
    """Validated active ID-to-black-edge-size mapping."""

    family: str
    sizes_m: Mapping[int, float]

    @classmethod
    def from_file(cls, path: str | Path) -> 'TagSizeRegistry':
        registry_path = Path(path)
        try:
            with registry_path.open('r', encoding='utf-8') as stream:
                raw = yaml.safe_load(stream)
        except (OSError, yaml.YAMLError) as exc:
            raise TagRegistryError(
                f'cannot load tag registry {registry_path}: {exc}'
            ) from exc
        return cls.from_mapping(raw)

    @classmethod
    def from_mapping(cls, raw) -> 'TagSizeRegistry':
        if not isinstance(raw, Mapping):
            raise TagRegistryError('tag registry root must be a mapping')
        family = raw.get('family')
        if not isinstance(family, str) or not family.strip():
            raise TagRegistryError('tag registry family must be a string')
        allocations = raw.get('allocations')
        if not isinstance(allocations, Sequence) or isinstance(
            allocations, (str, bytes)
        ):
            raise TagRegistryError('tag registry allocations must be a list')

        sizes_m: Dict[int, float] = {}
        allocated_ids = set()
        for index, allocation in enumerate(allocations):
            if not isinstance(allocation, Mapping):
                raise TagRegistryError(f'allocation {index} must be a mapping')
            first, last = _parse_id_range(allocation.get('ids'), index)
            current_ids = set(range(first, last + 1))
            overlap = allocated_ids.intersection(current_ids)
            if overlap:
                raise TagRegistryError(
                    f'allocation {index} overlaps active ID {min(overlap)}'
                )
            allocated_ids.update(current_ids)

            if 'tag_mm' not in allocation:
                continue
            tag_mm = _positive_float(
                allocation['tag_mm'], f'allocation {index} tag_mm'
            )
            black_edge_m = tag_mm * BLACK_EDGE_RATIO / MM_PER_M
            for tag_id in current_ids:
                sizes_m[tag_id] = black_edge_m
        return cls(family=family, sizes_m=sizes_m)

    def size_for(self, tag_id: int) -> Optional[float]:
        return self.sizes_m.get(int(tag_id))

    def reference_size(self) -> float:
        if not self.sizes_m:
            raise TagRegistryError('tag registry contains no pose-enabled IDs')
        return min(self.sizes_m.values())


@dataclass(frozen=True)
class TagSizeResolver:
    """Resolve one physical black-edge size without guessing."""

    registry: TagSizeRegistry
    single_size_m: Optional[float]
    per_id_sizes_m: Mapping[int, float]

    @classmethod
    def create(
        cls,
        registry: TagSizeRegistry,
        single_size_m=0.0,
        per_id_raw=(),
    ) -> 'TagSizeResolver':
        try:
            single_size = float(single_size_m)
        except (TypeError, ValueError) as exc:
            raise TagRegistryError('tag_size must be numeric') from exc
        per_id_sizes = parse_tag_size_overrides(per_id_raw)
        if not math.isfinite(single_size) or single_size < 0.0:
            raise TagRegistryError('tag_size must be zero or positive')
        if single_size > 0.0 and per_id_sizes:
            raise TagRegistryError(
                'tag_size and tag_sizes cannot be configured together'
            )
        return cls(
            registry=registry,
            single_size_m=single_size if single_size > 0.0 else None,
            per_id_sizes_m=per_id_sizes,
        )

    def size_for(self, tag_id: int) -> Optional[float]:
        if self.single_size_m is not None:
            return self.single_size_m
        if int(tag_id) in self.per_id_sizes_m:
            return self.per_id_sizes_m[int(tag_id)]
        return self.registry.size_for(tag_id)

    def reference_size(self) -> float:
        if self.single_size_m is not None:
            return self.single_size_m
        candidates = list(self.registry.sizes_m.values())
        candidates.extend(self.per_id_sizes_m.values())
        if not candidates:
            raise TagRegistryError('no tag size is available for pose estimation')
        return min(candidates)

    def mode_description(self) -> str:
        if self.single_size_m is not None:
            return f'single_override({self.single_size_m:.6f}m)'
        if self.per_id_sizes_m:
            return f'registry+{len(self.per_id_sizes_m)}_per_id_overrides'
        return 'registry'


def parse_tag_size_overrides(raw) -> Dict[int, float]:
    """Parse the ROS flat-list representation ``[id, metres, ...]``."""
    if raw is None:
        return {}
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)):
        raise TagRegistryError('tag_sizes must contain [id, size] pairs')
    if len(raw) == 0:
        return {}
    if len(raw) % 2 != 0:
        raise TagRegistryError('tag_sizes must contain [id, size] pairs')

    result: Dict[int, float] = {}
    for index in range(0, len(raw), 2):
        tag_id = _nonnegative_int(
            raw[index], f'tag_sizes ID at index {index}'
        )
        if tag_id in result:
            raise TagRegistryError(f'duplicate tag_sizes ID {tag_id}')
        result[tag_id] = _positive_float(
            raw[index + 1], f'tag_sizes size for ID {tag_id}'
        )
    return result


def _parse_id_range(raw, index: int) -> tuple[int, int]:
    if (
        not isinstance(raw, Sequence)
        or isinstance(raw, (str, bytes))
        or len(raw) != 2
    ):
        raise TagRegistryError(f'allocation {index} ids must be [first, last]')
    first = _nonnegative_int(raw[0], f'allocation {index} first ID')
    last = _nonnegative_int(raw[1], f'allocation {index} last ID')
    if last < first:
        raise TagRegistryError(f'allocation {index} has invalid ID range')
    return first, last


def _positive_float(raw, label: str) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise TagRegistryError(f'{label} must be numeric') from exc
    if not math.isfinite(value) or value <= 0.0:
        raise TagRegistryError(f'{label} must be positive')
    return value


def _nonnegative_int(raw, label: str) -> int:
    if isinstance(raw, bool):
        raise TagRegistryError(f'{label} must be an integer')
    try:
        value = int(raw)
        numeric = float(raw)
    except (TypeError, ValueError) as exc:
        raise TagRegistryError(f'{label} must be an integer') from exc
    if not math.isfinite(numeric) or numeric != value:
        raise TagRegistryError(f'{label} must be an integer')
    if value < 0:
        raise TagRegistryError(f'{label} must be non-negative')
    return value
