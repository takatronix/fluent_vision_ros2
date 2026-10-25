# PlaybackDone.msg
# audio/output/frame の1フレーム再生完了通知。
#
# - header: 元の AudioFrame.header をそのまま転送
# - request_id: 相関用ID（通常は header.frame_id のコピー）
# - epoch: AudioFrame.epoch のコピー（stop世代）

std_msgs/Header header
string request_id
uint32 epoch

# 足した項目 (古い出力からは既定値 = false / 空)
# complete: このフレームを途中で装置を失わずに全部書けた (drain_after_frame なら drain・prepare も成功)。false = 途中が
#           欠けたかもしれない (装置の切断・再接続、drain の失敗)
# drained: complete で、かつ snd_pcm_drain で鳴らし切ってから出した (audio.drain_after_frame=true)。true ならこの知らせの
#          時点でこのフレームの音は残っていない (後の stop で切れる音も無い)
# publisher_gid: この知らせを出した publisher の gid (受け手が ROS graph の endpoint_gid と照合し、どの出力の知らせか確かめる)
bool complete
bool drained
uint8[] publisher_gid
