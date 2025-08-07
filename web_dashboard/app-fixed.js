// StreamManager のaddStream関数を修正
// 元のapp.jsの該当部分を置き換え

document.getElementById('streamType').addEventListener('change', (e) => {
    const streamType = e.target.value;
    const urlGroup = document.getElementById('urlGroup');
    const rosbridgeGroup = document.getElementById('rosbridgeGroup');
    const rosTopicGroup = document.getElementById('rosTopicGroup');
    
    // すべて非表示にしてから必要なものだけ表示
    urlGroup.style.display = 'none';
    rosbridgeGroup.style.display = 'none';
    rosTopicGroup.style.display = 'none';
    
    switch(streamType) {
        case 'ros':
        case 'pointcloud':
            // ROS関連はROSBridgeとTopicのみ表示
            rosbridgeGroup.style.display = 'block';
            rosTopicGroup.style.display = 'block';
            break;
            
        case 'mjpeg':
        case 'websocket':
        case 'webrtc':
        case 'rtmp':
        case 'iframe':
            // その他はURLのみ表示
            urlGroup.style.display = 'block';
            break;
    }
});

// addStream関数も修正
function addStream() {
    const name = document.getElementById('streamName').value || 'Unnamed Stream';
    const type = document.getElementById('streamType').value;
    const streamId = `stream-${Date.now()}`;
    
    let streamConfig = {
        id: streamId,
        name: name,
        type: type,
        status: 'connecting'
    };
    
    // タイプに応じて必要な情報のみ取得
    if (type === 'ros' || type === 'pointcloud') {
        // ROS系はROSBridgeとTopicのみ
        streamConfig.rosbridge = document.getElementById('rosbridgeUrl').value;
        streamConfig.rosTopic = document.getElementById('rosTopic').value;
        
        if (!streamConfig.rosTopic) {
            alert('ROS Topicを入力してください');
            return;
        }
    } else {
        // その他はURLのみ
        streamConfig.url = document.getElementById('streamUrl').value;
        
        if (!streamConfig.url) {
            alert('URLを入力してください');
            return;
        }
    }
    
    this.createStreamTile(streamConfig);
    this.streams.set(streamId, streamConfig);
    
    // フォームをリセット
    document.getElementById('streamName').value = '';
    document.getElementById('streamUrl').value = '';
    document.getElementById('rosTopic').value = '';
    
    this.closeSidePanel();
    
    // ストリームへの接続を開始
    this.connectStream(streamConfig);
}