# fv_soundboard

`/fv/event` (`std_msgs/String`) の登録名を固定の SYSTEM 発話と cue PCM に解決します。

- SYSTEM 発話要求: `/aspa/dialogue/tts/request`
- cue PCM: `/audio/cue/frame`

物理デバイスへは接続しません。両出力は `fv_audio/playback_controller` が単一の再生ストリームへ統合します。
`/fv/event` は bare 名または `{"event":"name"}` のみを受け付け、発話文や cue の入力時上書きはできません。
通常イベントは音の有効設定にかかわらず先に `/fv/event/active` へ記録されます。
`/fv/sound/settings` は TRANSIENT_LOCAL な次の共有設定を受け付けます。

```json
{"version":1,"robot_enabled":true,"disabled_events":[]}
```

`/fv/sound/play` は登録イベント名のプレビュー専用です。共有設定を迂回して再生しますが、
`/fv/event/active` へは記録しません。

読み上げ要求は、Scene Viewerが永続化して配信する
`/aspa/voice/maintenance` を受信した後だけ発行します。保守中または状態未受信時は
volatileな要求をDialogue不在のトピックへ流さず、同じrequest ID/utterance IDを持つ
`TtsResult.REJECTED` を `/aspa/tts/result` へ発行します。受信した状態は
`/fv/soundboard/voice-maintenance` にtransient-localで確認応答し、
`scripts/restart-voice` はこの確認応答を見るまで音声プロセスを停止しません。

```bash
ros2 launch fv_soundboard fv_soundboard.launch.py
ros2 topic pub -1 /fv/event std_msgs/msg/String "{data: ready}"
```
