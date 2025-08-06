# 録画状態メッセージ
std_msgs/Header header

# 録画状態
bool is_recording            # 録画中フラグ
bool is_playing              # 再生中フラグ
string recording_id          # 録画ID
string playback_id           # 再生ID

# ファイル情報
string current_file_path     # 現在のファイルパス
int64 file_size_bytes        # ファイルサイズ（バイト）
float64 recording_duration   # 録画時間（秒）

# 統計情報
int32 total_frames_recorded  # 録画フレーム数
int32 total_frames_played    # 再生フレーム数
float64 fps                  # フレームレート 