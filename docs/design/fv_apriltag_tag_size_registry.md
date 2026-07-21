# fv_apriltag runtime tag-size contract

`fv_apriltag` estimates poses from the black AprilTag square, not from the
nominal paper edge.  The runtime source of truth is
`config/tag_registry.yaml`; an allocation with `tag_mm` therefore resolves to
`tag_mm * 0.8 / 1000` metres.

Resolution order and failure behaviour:

1. A positive `tag_size` parameter is an explicit homogeneous-size override
   for every detected ID.  It is intended for a controlled board or legacy
   deployment where every visible physical tag was measured to that black-edge
   size.
2. Otherwise an ID in `tag_sizes` overrides its registry allocation.
3. Otherwise the active `allocations` entry in `tag_registry.yaml` supplies
   the size.  `legacy` entries are documentation only and are not enabled.
4. Unknown IDs and allocations without `tag_mm` (reserved bands) have no pose.
   They are omitted from both `detections` and `poses`, preserving the
   one-to-one array contract used by `cube_estimator_node`; no TF is published.

`tag_size` and `tag_sizes` cannot be set together because their precedence
would be ambiguous.  Invalid or overlapping active allocations, non-positive
sizes, and a registry/detector family mismatch stop node construction rather
than silently selecting a plausible size.

The former `default_tag_size` parameter is intentionally not retained as a
compatibility fallback: its implicit 0.050 m value was neither the 50 mm
print's 0.040 m black edge nor safe for mixed-size field tags.  A deployment
that truly has one measured size must migrate to the explicit `tag_size`
parameter.  Size parameters and `family` are startup-only so a parameter
update cannot appear successful without rebuilding the detector/resolver.

The `piper.launch.py` and `daihen.launch.py` cube profiles explicitly set
`tag_size: 0.040` because their still-deployed legacy cube IDs overlap the
current dock/navigation bands.  This override is intentionally local to those
homogeneous cube profiles; generic and field deployments must use the active
registry and must never infer legacy meaning from an ID alone.
