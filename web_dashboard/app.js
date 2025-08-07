// FluentVision - Multi-Stream Dashboard Application

class StreamManager {
    constructor() {
        this.streams = new Map();
        this.layouts = {
            single: { cols: 1, rows: 1 },
            quad: { cols: 2, rows: 2 },
            six: { cols: 3, rows: 2 },
            custom: null
        };
        this.currentLayout = 'quad';
        this.init();
    }

    init() {
        this.setupEventListeners();
        this.loadSavedLayout();
        this.addDemoStreams();
    }

    setupEventListeners() {
        // レイアウトボタン
        document.querySelectorAll('.layout-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                this.changeLayout(e.target.dataset.layout);
            });
        });

        // ストリーム追加ボタン
        document.getElementById('addStreamBtn').addEventListener('click', () => {
            this.openSidePanel();
        });

        // サイドパネルの閉じるボタン
        document.querySelector('.close-btn').addEventListener('click', () => {
            this.closeSidePanel();
        });

        // ストリーム追加確認ボタン
        document.getElementById('addStreamConfirm').addEventListener('click', () => {
            this.addStream();
        });

        // ストリームタイプ変更時の処理
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

        // レイアウト保存
        document.getElementById('saveLayoutBtn').addEventListener('click', () => {
            this.saveLayout();
        });

        // フルスクリーン
        document.getElementById('fullscreenBtn').addEventListener('click', () => {
            this.toggleFullscreen();
        });

        // ドラッグ&ドロップの設定
        this.setupDragAndDrop();
    }

    changeLayout(layout) {
        this.currentLayout = layout;
        const grid = document.getElementById('streamGrid');
        
        // レイアウトクラスを更新
        grid.className = `stream-grid layout-${layout}`;
        
        // ボタンのアクティブ状態を更新
        document.querySelectorAll('.layout-btn').forEach(btn => {
            btn.classList.remove('active');
        });
        document.querySelector(`[data-layout="${layout}"]`).classList.add('active');
    }

    openSidePanel() {
        document.getElementById('sidePanel').classList.add('open');
    }

    closeSidePanel() {
        document.getElementById('sidePanel').classList.remove('open');
    }

    addStream() {
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

    createStreamTile(config) {
        const template = document.getElementById('streamTileTemplate');
        const tile = template.content.cloneNode(true);
        
        const tileElement = tile.querySelector('.stream-tile');
        tileElement.id = config.id;
        tileElement.querySelector('.tile-title').textContent = config.name;
        
        // 閉じるボタン
        tileElement.querySelector('.close-tile').addEventListener('click', () => {
            this.removeStream(config.id);
        });
        
        // 最大化ボタン
        tileElement.querySelector('.fullscreen-tile').addEventListener('click', () => {
            this.maximizeTile(config.id);
        });
        
        // ストリームコンテナに適切な要素を追加
        const container = tileElement.querySelector('.stream-container');
        this.createStreamElement(container, config);
        
        // グリッドに追加
        document.getElementById('streamGrid').appendChild(tileElement);
        
        // アニメーション
        setTimeout(() => {
            tileElement.classList.add('new');
        }, 10);
    }

    createStreamElement(container, config) {
        let element;
        
        switch(config.type) {
            case 'mjpeg':
                element = document.createElement('img');
                element.src = config.url;
                break;
                
            case 'websocket':
                element = document.createElement('canvas');
                // WebSocketストリームの処理を後で実装
                break;
                
            case 'webrtc':
                element = document.createElement('video');
                element.autoplay = true;
                element.muted = true;
                // WebRTC接続の処理を後で実装
                break;
                
            case 'rtmp':
                // HLS.jsを使用してRTMP/HLSストリームを再生
                element = document.createElement('video');
                element.autoplay = true;
                element.muted = true;
                this.setupHLSPlayer(element, config.url);
                break;
                
            case 'ros':
                element = document.createElement('canvas');
                // ROSブリッジ接続の処理を後で実装
                break;
                
            case 'iframe':
                element = document.createElement('iframe');
                element.src = config.url;
                element.frameBorder = '0';
                break;
                
            default:
                element = document.createElement('div');
                element.textContent = 'Unsupported stream type';
        }
        
        container.appendChild(element);
    }

    connectStream(config) {
        const tileElement = document.getElementById(config.id);
        const statusIndicator = tileElement.querySelector('.status-indicator');
        const statusText = tileElement.querySelector('.status-text');
        
        // 接続シミュレーション（実際の実装では各プロトコルに応じた接続処理）
        setTimeout(() => {
            statusIndicator.classList.add('connected');
            statusText.textContent = '接続済み';
            config.status = 'connected';
        }, 1000);
    }

    removeStream(streamId) {
        const tile = document.getElementById(streamId);
        if (tile) {
            tile.remove();
            this.streams.delete(streamId);
        }
    }

    maximizeTile(streamId) {
        const tile = document.getElementById(streamId);
        if (tile.classList.contains('maximized')) {
            // 元のサイズに戻す
            tile.classList.remove('maximized');
            tile.style.position = '';
            tile.style.zIndex = '';
            tile.style.width = '';
            tile.style.height = '';
        } else {
            // 最大化
            tile.classList.add('maximized');
            tile.style.position = 'fixed';
            tile.style.top = '0';
            tile.style.left = '0';
            tile.style.width = '100vw';
            tile.style.height = '100vh';
            tile.style.zIndex = '1000';
        }
    }

    setupDragAndDrop() {
        let draggedElement = null;
        
        document.addEventListener('dragstart', (e) => {
            if (e.target.classList.contains('stream-tile')) {
                draggedElement = e.target;
                e.target.classList.add('dragging');
            }
        });
        
        document.addEventListener('dragend', (e) => {
            if (e.target.classList.contains('stream-tile')) {
                e.target.classList.remove('dragging');
            }
        });
        
        document.addEventListener('dragover', (e) => {
            e.preventDefault();
            const afterElement = this.getDragAfterElement(e.clientY);
            const grid = document.getElementById('streamGrid');
            
            if (afterElement == null) {
                grid.appendChild(draggedElement);
            } else {
                grid.insertBefore(draggedElement, afterElement);
            }
        });
        
        document.addEventListener('drop', (e) => {
            e.preventDefault();
        });
    }

    getDragAfterElement(y) {
        const draggableElements = [...document.querySelectorAll('.stream-tile:not(.dragging)')];
        
        return draggableElements.reduce((closest, child) => {
            const box = child.getBoundingClientRect();
            const offset = y - box.top - box.height / 2;
            
            if (offset < 0 && offset > closest.offset) {
                return { offset: offset, element: child };
            } else {
                return closest;
            }
        }, { offset: Number.NEGATIVE_INFINITY }).element;
    }

    saveLayout() {
        const layout = {
            type: this.currentLayout,
            streams: Array.from(this.streams.values()),
            positions: this.getStreamPositions()
        };
        
        localStorage.setItem('fluentvision-layout', JSON.stringify(layout));
        this.showNotification('レイアウトを保存しました');
    }

    loadSavedLayout() {
        const saved = localStorage.getItem('fluentvision-layout');
        if (saved) {
            try {
                const layout = JSON.parse(saved);
                this.changeLayout(layout.type);
                
                // 保存されたストリームを復元
                layout.streams.forEach(stream => {
                    this.createStreamTile(stream);
                    this.streams.set(stream.id, stream);
                    this.connectStream(stream);
                });
            } catch (e) {
                console.error('Failed to load saved layout:', e);
            }
        }
    }

    getStreamPositions() {
        const positions = {};
        document.querySelectorAll('.stream-tile').forEach((tile, index) => {
            positions[tile.id] = index;
        });
        return positions;
    }

    toggleFullscreen() {
        if (!document.fullscreenElement) {
            document.documentElement.requestFullscreen();
            document.body.classList.add('fullscreen');
        } else {
            document.exitFullscreen();
            document.body.classList.remove('fullscreen');
        }
    }

    showNotification(message) {
        // 通知を表示（後で実装）
        console.log(message);
    }

    setupHLSPlayer(video, url) {
        // HLS.jsを使用した実装（CDNから読み込む必要あり）
        if (typeof Hls !== 'undefined' && Hls.isSupported()) {
            const hls = new Hls();
            hls.loadSource(url);
            hls.attachMedia(video);
        } else if (video.canPlayType('application/vnd.apple.mpegurl')) {
            video.src = url;
        }
    }

    // デモストリームを追加
    addDemoStreams() {
        const demoStreams = [
            {
                id: 'demo-1',
                name: 'ROS2 Camera D415',
                type: 'mjpeg',
                url: 'http://localhost:8080/stream?topic=/fv/d415/color/image_raw',
                status: 'connecting'
            },
            {
                id: 'demo-2', 
                name: 'M5Stack Camera',
                type: 'mjpeg',
                url: 'http://192.168.1.100/stream',
                status: 'connecting'
            },
            {
                id: 'demo-3',
                name: 'System Monitor',
                type: 'iframe',
                url: 'https://example.com/dashboard',
                status: 'connecting'
            },
            {
                id: 'demo-4',
                name: 'WebRTC Stream',
                type: 'webrtc',
                url: 'ws://localhost:8080/webrtc',
                status: 'connecting'
            }
        ];

        // デモストリームを順番に追加
        demoStreams.forEach((stream, index) => {
            setTimeout(() => {
                this.createStreamTile(stream);
                this.streams.set(stream.id, stream);
                this.connectStream(stream);
            }, index * 200);
        });
    }
}

// WebSocket接続マネージャー
class WebSocketStreamManager {
    constructor() {
        this.connections = new Map();
    }

    connect(streamId, url) {
        const ws = new WebSocket(url);
        
        ws.onopen = () => {
            console.log(`WebSocket connected: ${streamId}`);
            this.connections.set(streamId, ws);
        };
        
        ws.onmessage = (event) => {
            // 画像データを処理してCanvasに描画
            this.handleStreamData(streamId, event.data);
        };
        
        ws.onerror = (error) => {
            console.error(`WebSocket error: ${streamId}`, error);
        };
        
        ws.onclose = () => {
            console.log(`WebSocket closed: ${streamId}`);
            this.connections.delete(streamId);
        };
    }

    handleStreamData(streamId, data) {
        const tile = document.getElementById(streamId);
        if (!tile) return;
        
        const canvas = tile.querySelector('canvas');
        if (!canvas) return;
        
        const ctx = canvas.getContext('2d');
        
        // Base64エンコードされた画像データの場合
        if (typeof data === 'string' && data.startsWith('data:image')) {
            const img = new Image();
            img.onload = () => {
                canvas.width = img.width;
                canvas.height = img.height;
                ctx.drawImage(img, 0, 0);
            };
            img.src = data;
        }
        // Blobデータの場合
        else if (data instanceof Blob) {
            const url = URL.createObjectURL(data);
            const img = new Image();
            img.onload = () => {
                canvas.width = img.width;
                canvas.height = img.height;
                ctx.drawImage(img, 0, 0);
                URL.revokeObjectURL(url);
            };
            img.src = url;
        }
    }

    disconnect(streamId) {
        const ws = this.connections.get(streamId);
        if (ws) {
            ws.close();
            this.connections.delete(streamId);
        }
    }
}

// WebRTC接続マネージャー
class WebRTCStreamManager {
    constructor() {
        this.peerConnections = new Map();
    }

    async connect(streamId, signalingUrl) {
        const pc = new RTCPeerConnection({
            iceServers: [
                { urls: 'stun:stun.l.google.com:19302' }
            ]
        });

        this.peerConnections.set(streamId, pc);

        // Signalingサーバーへの接続
        const ws = new WebSocket(signalingUrl);
        
        ws.onopen = () => {
            console.log(`WebRTC signaling connected: ${streamId}`);
        };

        pc.ontrack = (event) => {
            const tile = document.getElementById(streamId);
            if (!tile) return;
            
            const video = tile.querySelector('video');
            if (video && event.streams[0]) {
                video.srcObject = event.streams[0];
            }
        };

        // ICE候補の処理
        pc.onicecandidate = (event) => {
            if (event.candidate) {
                ws.send(JSON.stringify({
                    type: 'ice-candidate',
                    candidate: event.candidate
                }));
            }
        };

        // Signalingメッセージの処理
        ws.onmessage = async (event) => {
            const message = JSON.parse(event.data);
            
            if (message.type === 'offer') {
                await pc.setRemoteDescription(message.offer);
                const answer = await pc.createAnswer();
                await pc.setLocalDescription(answer);
                
                ws.send(JSON.stringify({
                    type: 'answer',
                    answer: answer
                }));
            } else if (message.type === 'ice-candidate') {
                await pc.addIceCandidate(message.candidate);
            }
        };
    }

    disconnect(streamId) {
        const pc = this.peerConnections.get(streamId);
        if (pc) {
            pc.close();
            this.peerConnections.delete(streamId);
        }
    }
}

// ROS Bridge接続マネージャー
class ROSBridgeManager {
    constructor() {
        this.ros = null;
        this.topics = new Map();
    }

    connect(url = 'ws://localhost:9090') {
        if (typeof ROSLIB === 'undefined') {
            console.error('ROSLIB not loaded. Please include roslib.js');
            return;
        }

        this.ros = new ROSLIB.Ros({
            url: url
        });

        this.ros.on('connection', () => {
            console.log('Connected to ROS bridge');
        });

        this.ros.on('error', (error) => {
            console.error('ROS bridge error:', error);
        });

        this.ros.on('close', () => {
            console.log('ROS bridge connection closed');
        });
    }

    subscribeToImageTopic(streamId, topicName) {
        if (!this.ros) {
            console.error('ROS not connected');
            return;
        }

        const topic = new ROSLIB.Topic({
            ros: this.ros,
            name: topicName,
            messageType: 'sensor_msgs/CompressedImage'
        });

        topic.subscribe((message) => {
            this.handleROSImage(streamId, message);
        });

        this.topics.set(streamId, topic);
    }

    handleROSImage(streamId, message) {
        const tile = document.getElementById(streamId);
        if (!tile) return;

        const canvas = tile.querySelector('canvas');
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        const img = new Image();
        
        img.onload = () => {
            canvas.width = img.width;
            canvas.height = img.height;
            ctx.drawImage(img, 0, 0);
        };
        
        img.src = 'data:image/jpeg;base64,' + message.data;
    }

    unsubscribe(streamId) {
        const topic = this.topics.get(streamId);
        if (topic) {
            topic.unsubscribe();
            this.topics.delete(streamId);
        }
    }
}

// アプリケーション初期化
document.addEventListener('DOMContentLoaded', () => {
    window.streamManager = new StreamManager();
    window.wsManager = new WebSocketStreamManager();
    window.webrtcManager = new WebRTCStreamManager();
    window.rosManager = new ROSBridgeManager();
    
    // ROS Bridgeへの自動接続（オプション）
    // rosManager.connect('ws://localhost:9090');
});