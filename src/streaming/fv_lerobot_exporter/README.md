# fv_lerobot_exporter

FluentVision recorderの確定済みepisodeをLeRobot v3 datasetへ変換するpure Python packageである。

録画制御、HTTP API、job管理、外部storage同期はこのpackageの対象外である。

変換アルゴリズムと入出力契約は[変換仕様](docs/lerobot_dataset_export.md)を正本とする。

## Public API

```python
from fv_lerobot_exporter import (
    LerobotDatasetExportRequest,
    export_lerobot_dataset,
)

result = export_lerobot_dataset(
    request=LerobotDatasetExportRequest(
        dataset_id="dataset-id",
        dataset_name="dataset-name",
        episode_ids=["episode-id"],
    ),
    profile_payload=profile_payload,
    datasets_dir=datasets_dir,
)
```
