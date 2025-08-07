// FluentVision - Dashboard Examples
// 様々なタイプのダッシュボードを追加するためのヘルパー

class DashboardExamples {
    constructor() {
        this.examples = {
            // 映像ストリーム
            cameras: [
                {
                    name: "📷 IPカメラ (MJPEG)",
                    type: "mjpeg",
                    url: "http://192.168.1.100/stream",
                    description: "一般的なIPカメラ"
                },
                {
                    name: "🤖 ROS2 Camera",
                    type: "ros",
                    rosbridge: "ws://localhost:9090",
                    topic: "/camera/image_raw/compressed",
                    description: "ROS2カメラトピック"
                },
                {
                    name: "📹 ESP32-CAM",
                    type: "mjpeg",
                    url: "http://192.168.1.60:81/stream",
                    description: "ESP32カメラモジュール"
                },
                {
                    name: "🎥 M5Stack UnitCam",
                    type: "mjpeg",
                    url: "http://192.168.1.50/stream",
                    description: "M5Stack UnitCam S3"
                }
            ],

            // 3Dビジュアライゼーション
            visualization: [
                {
                    name: "☁️ PointCloud",
                    type: "pointcloud",
                    rosbridge: "ws://localhost:9090",
                    topic: "/camera/depth/points",
                    description: "3D点群表示"
                },
                {
                    name: "📊 3D Chart",
                    type: "custom",
                    component: "ThreeJSChart",
                    description: "3Dグラフ表示"
                }
            ],

            // Webダッシュボード
            dashboards: [
                {
                    name: "📈 Grafana",
                    type: "iframe",
                    url: "http://localhost:3000",
                    description: "メトリクス可視化"
                },
                {
                    name: "🏠 Home Assistant",
                    type: "iframe",
                    url: "http://localhost:8123",
                    description: "スマートホーム"
                },
                {
                    name: "🔴 Node-RED",
                    type: "iframe",
                    url: "http://localhost:1880",
                    description: "フロー制御"
                },
                {
                    name: "📊 Netdata",
                    type: "iframe",
                    url: "http://localhost:19999",
                    description: "システム監視"
                },
                {
                    name: "🐳 Portainer",
                    type: "iframe",
                    url: "http://localhost:9000",
                    description: "Docker管理"
                }
            ],

            // センサーデータ
            sensors: [
                {
                    name: "🌡️ 温度センサー",
                    type: "websocket",
                    url: "ws://localhost:8080/temperature",
                    description: "リアルタイム温度"
                },
                {
                    name: "📡 LiDAR Scan",
                    type: "ros",
                    rosbridge: "ws://localhost:9090",
                    topic: "/scan",
                    description: "2D LiDARデータ"
                },
                {
                    name: "🎯 IMU Data",
                    type: "ros",
                    rosbridge: "ws://localhost:9090",
                    topic: "/imu/data",
                    description: "加速度・ジャイロ"
                }
            ],

            // カスタムコンポーネント
            custom: [
                {
                    name: "🗺️ Map View",
                    type: "custom",
                    component: "MapboxViewer",
                    description: "地図表示"
                },
                {
                    name: "🎮 Joystick",
                    type: "custom",
                    component: "VirtualJoystick",
                    description: "仮想ジョイスティック"
                },
                {
                    name: "📱 QR Scanner",
                    type: "custom",
                    component: "QRCodeScanner",
                    description: "QRコードスキャナー"
                }
            ]
        };
    }

    // カテゴリー別のダッシュボードリストを取得
    getByCategory(category) {
        return this.examples[category] || [];
    }

    // すべてのダッシュボードを取得
    getAll() {
        const all = [];
        Object.values(this.examples).forEach(category => {
            all.push(...category);
        });
        return all;
    }

    // URLパターンから自動判定
    autoDetectType(url) {
        if (url.includes(':81/stream') || url.includes('/stream')) {
            return 'mjpeg';
        }
        if (url.startsWith('ws://') || url.startsWith('wss://')) {
            if (url.includes('9090')) {
                return 'ros';
            }
            return 'websocket';
        }
        if (url.startsWith('rtmp://') || url.includes('.m3u8')) {
            return 'rtmp';
        }
        if (url.startsWith('http://') || url.startsWith('https://')) {
            return 'iframe';
        }
        return 'unknown';
    }

    // プリセットレイアウトを生成
    generateLayout(preset) {
        const layouts = {
            // ロボット監視レイアウト
            robot: [
                { title: "Front Camera", type: "mjpeg", x: 10, y: 60, width: 400, height: 300 },
                { title: "Rear Camera", type: "mjpeg", x: 420, y: 60, width: 400, height: 300 },
                { title: "PointCloud", type: "pointcloud", x: 830, y: 60, width: 400, height: 300 },
                { title: "System Stats", type: "iframe", url: "http://localhost:19999", x: 10, y: 370, width: 600, height: 200 },
                { title: "Control Panel", type: "iframe", url: "http://localhost:1880", x: 620, y: 370, width: 610, height: 200 }
            ],

            // スマートホーム
            home: [
                { title: "Home Assistant", type: "iframe", url: "http://localhost:8123", x: 10, y: 60, width: 600, height: 500 },
                { title: "Living Room", type: "mjpeg", x: 620, y: 60, width: 300, height: 240 },
                { title: "Front Door", type: "mjpeg", x: 930, y: 60, width: 300, height: 240 },
                { title: "Temperature", type: "custom", component: "TemperatureGauge", x: 620, y: 310, width: 300, height: 250 },
                { title: "Energy Usage", type: "iframe", url: "http://localhost:3000", x: 930, y: 310, width: 300, height: 250 }
            ],

            // 開発環境
            development: [
                { title: "Logs", type: "websocket", url: "ws://localhost:8080/logs", x: 10, y: 60, width: 400, height: 500 },
                { title: "Metrics", type: "iframe", url: "http://localhost:3000", x: 420, y: 60, width: 400, height: 250 },
                { title: "Docker", type: "iframe", url: "http://localhost:9000", x: 830, y: 60, width: 400, height: 250 },
                { title: "API Docs", type: "iframe", url: "http://localhost:8000/docs", x: 420, y: 320, width: 810, height: 240 }
            ],

            // ドローン操作
            drone: [
                { title: "FPV Camera", type: "webrtc", x: 10, y: 60, width: 800, height: 450 },
                { title: "Map", type: "custom", component: "MapView", x: 820, y: 60, width: 410, height: 450 },
                { title: "Telemetry", type: "websocket", x: 10, y: 520, width: 400, height: 100 },
                { title: "Battery", type: "custom", component: "BatteryIndicator", x: 420, y: 520, width: 200, height: 100 },
                { title: "Controls", type: "custom", component: "DroneControls", x: 630, y: 520, width: 600, height: 100 }
            ]
        };

        return layouts[preset] || [];
    }
}

// カスタムコンポーネントの基本テンプレート
class CustomDashboardComponent {
    constructor(container, config) {
        this.container = container;
        this.config = config;
        this.init();
    }

    init() {
        // オーバーライドして実装
    }

    update(data) {
        // データ更新時の処理
    }

    dispose() {
        // クリーンアップ処理
    }
}

// 温度ゲージの例
class TemperatureGauge extends CustomDashboardComponent {
    init() {
        this.container.innerHTML = `
            <div style="width:100%;height:100%;display:flex;flex-direction:column;align-items:center;justify-content:center;color:white;">
                <div style="font-size:3rem;font-weight:bold;">
                    <span id="temp-value">--</span>°C
                </div>
                <div style="width:80%;height:20px;background:rgba(255,255,255,0.1);border-radius:10px;margin-top:20px;">
                    <div id="temp-bar" style="width:50%;height:100%;background:linear-gradient(90deg,#4ade80,#f59e0b,#ef4444);border-radius:10px;transition:width 0.5s;"></div>
                </div>
                <div style="margin-top:10px;opacity:0.6;">室温</div>
            </div>
        `;
        
        // デモ用のランダム更新
        setInterval(() => {
            const temp = 20 + Math.random() * 10;
            this.update(temp);
        }, 2000);
    }

    update(temperature) {
        const valueEl = this.container.querySelector('#temp-value');
        const barEl = this.container.querySelector('#temp-bar');
        
        if (valueEl) valueEl.textContent = temperature.toFixed(1);
        if (barEl) barEl.style.width = `${Math.min(100, (temperature / 40) * 100)}%`;
    }
}

// グローバルに公開
window.DashboardExamples = DashboardExamples;
window.CustomDashboardComponent = CustomDashboardComponent;
window.TemperatureGauge = TemperatureGauge;