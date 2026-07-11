# Episode metadata契約の保留事項

このメモは未決事項だけを記録する。

現行exporterの入力契約として扱わない。

## 録画時profile snapshot

episode metadataはprofile名だけを保持している。

録画後に同名profileのLeRobot設定が変わると、変換時に解決した設定と録画時の設定が一致しない可能性がある。

録画開始時に解決済みprofileをsnapshotするかどうか、その保存範囲と更新規則は未決である。

## Trim指定

`trim_start_s`と`trim_end_s`をepisode metadataまたはexport requestへ持たせるかどうかは未決である。

現行exporterに両fieldは存在せず、camera sidecarと必須state sampleから固定FPS gridの範囲を求める。

trimを導入する場合は、実データから求めたgridとの関係、許容範囲、provenanceへの記録方法を先に決める。
