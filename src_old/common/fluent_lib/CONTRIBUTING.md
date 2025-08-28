# FluentLib Conventions (Short code is justice)

- Naming
  - Namespaces/paths: `fluent_lib`, domain subspaces: `fluent_cloud`, `fluent_image`, `fluent_ui`, `fluent_lib::ros`
  - Classes/structs/enums: PascalCase (e.g., Image, Renderer, Worker, Sequence, Ease, PCAMetrics)
  - Free functions/utilities: lower_snake_case (e.g., compute_pca_metrics, voxel_sor, move_to, fade_in)
  - Methods: lowerCamelCase verbs (e.g., setPos, setAlpha)
  - DSL entry points may use PascalCase for readability (e.g., FluentPubImage)

- Parameters/Topics
  - Never hardcode ROS topics in code. Use parameter names (strings) only.
  - YAML defines actual topic strings and UI strings.

- Legacy
  - Do not include `legacy/*` directly. Use FluentCloud/FluentImage/FluentUI/ROS helpers.

- UI/Animation
  - Prefer YAML-defined resources + keyword substitution; code passes values only.
  - Use fluent_ui animation helpers (move_to, fade_in/out, Sequence chaining).

- Images/PointClouds
  - Use `fluent_image::Image` (implicit cv::Mat/ROS Image conversions) to hide encodings.
  - Use FluentCloud for filters/metrics; prefer `compute_pca_metrics`, `VoxelGrid`, `StatisticalOutlierRemoval`.

- ROS helpers
  - Use `fluent_lib::ros` helpers for param/pub/sub/timer/log/qos.
  - Prefer readable overloads (pub_image/sub_image/sub_fimage/sub_cloud) over templates in call sites.

- Style
  - Keep call sites minimal and readable; hide boilerplate behind helpers.
  - No comments explaining trivial code; document "why", not "how".
