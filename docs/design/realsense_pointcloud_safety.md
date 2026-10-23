# RealSense点群のメモリ境界と座標メタデータ修正

結果: organized点群の確保サイズ、RGB参照、点群の座標系・時刻を修正する提案。

## 不具合と変更

- `organized_pointcloud.decimation=3`、640×480では、ループは214×160点を生成するが従来の確保は213×160点。XYZの場合1,920バイト、XYZRGBの場合2,560バイト不足し、ヒープを破壊する。軸ごとの標本数を切り上げて確保する。画像より大きい間引き幅も1点として扱う。
- 深度の画素位置でカラー画像を直接読むため、深度1280×720・カラー640×480では領域外アクセスになる。同解像度でも外部・内部パラメータが異なれば着色位置がずれる。SDKの深度→カラー外部パラメータで3D点を変換し、カラー内部パラメータで投影して有限値・正のZ・画像境界を確認する。カラー画角外は黒とし、XYZ自体は保持する。遮蔽判定は追加していない。
- XYZは深度内部パラメータで逆投影しているため、通常点群・organized点群とも深度optical frameを宣言する。通常点群のstampは深度画像と同じ取得時刻マッピングを引き継ぎ、処理完了時のsystem clockで上書きしない。
- 通常点群は購読者ゼロなら生成を省略する。

`streams.align_to_color` は現実装で画像再投影を実行していない。この変更はその機能を実装せず、organized点群のフレーム名だけがカラーになる誤りを除く。色付けの空間投影と深度画像のカラー整列は別の処理である。

## ハードウェア不要の検証

リポジトリルートで実行:

```sh
bash src/sensors/fv_realsense/test/test_pointcloud_geometry.sh
python3 src/sensors/fv_realsense/test/test_pointcloud_wiring.py
bash src/sensors/fv_realsense/test/test_fv_uvcvideo.sh
```

- geometry: ASan/UBSan付きで240通りの画像寸法・間引き幅・XYZ/XYZRGB配置の書き込みを検査。非有限値、画像境界、極端なパラメータも検査する。
- wiring: 本番のRGBサンプラ関数をソースから抽出してコンパイル。OpenCV/SDKのテストダブルで、非ゼロの並進を用いた投影、画角外・背面・非有限値の処理を検査する。ソース検査でフレーム名・時刻引き渡しと両点群経路の接続も確認する。
- 旧処理の縮約C++再現では、640×480/step=3の書き込みと、深度1280×720/カラー640×480の読み込みがそれぞれASanのheap-buffer-overflowになることを確認した。

検証環境にはROS 2/OpenCV/RealSense SDKの開発環境がない。ノード全体のビルド、SDKによる歪み補正、実機のプロファイル・TF・時刻同期は未確認。テストダブルはSDK自体の正しさを検証しない。

## 残課題

- 内部TF broadcasterはカメラ・カラーoptical・深度optical間を恒等変換で配信する。機体側TFとの責任分担、optical軸変換、工場外部パラメータを別途整合させる必要がある。
- depth処理は最新のカラーを取得するだけで、`sync_max_skew_ms`による組み合わせ制限を実行していない。空間投影の修正だけでは動く物体の時間ずれを解決しない。
- GetDistanceは深度画素を逆投影するがcolor optical frameを返す。要求の画素座標系・frame_idの契約を整理して修正する必要がある。
