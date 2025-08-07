// モバイルデバイス用の設定ヘルパー

class MobileConfig {
    constructor() {
        this.isMobile = this.detectMobile();
        this.serverIP = this.getServerIP();
    }

    detectMobile() {
        return /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
    }

    getServerIP() {
        // URLから自動的にサーバーIPを取得
        const hostname = window.location.hostname;
        
        // ローカルホストの場合は、ユーザーに入力を求める
        if (hostname === 'localhost' || hostname === '127.0.0.1') {
            return this.promptForServerIP();
        }
        
        return hostname;
    }

    promptForServerIP() {
        // LocalStorageから前回のIPを取得
        const savedIP = localStorage.getItem('ros2_server_ip');
        
        if (savedIP) {
            const usesSaved = confirm(`前回のサーバー (${savedIP}) を使用しますか？`);
            if (usesSaved) return savedIP;
        }
        
        const ip = prompt('ROS2サーバーのIPアドレスを入力してください:', '192.168.1.100');
        if (ip) {
            localStorage.setItem('ros2_server_ip', ip);
        }
        return ip || 'localhost';
    }

    getROSBridgeURL() {
        return `ws://${this.serverIP}:9090`;
    }

    getMJPEGBaseURL() {
        return `http://${this.serverIP}:8080`;
    }

    getWebRTCSignalingURL() {
        return `ws://${this.serverIP}:8088`;
    }

    // モバイル最適化設定を適用
    applyMobileOptimizations() {
        if (!this.isMobile) return;

        // タッチジェスチャーを有効化
        this.enableTouchGestures();
        
        // パフォーマンス最適化
        this.optimizePerformance();
        
        // UIの調整
        this.adjustUI();
    }

    enableTouchGestures() {
        let touchStartX = 0;
        let touchStartY = 0;
        let currentElement = null;

        // ドラッグ&ドロップをタッチ対応に
        document.addEventListener('touchstart', (e) => {
            const tile = e.target.closest('.stream-tile');
            if (tile) {
                currentElement = tile;
                touchStartX = e.touches[0].clientX;
                touchStartY = e.touches[0].clientY;
                tile.style.opacity = '0.8';
            }
        });

        document.addEventListener('touchmove', (e) => {
            if (currentElement) {
                e.preventDefault();
                const touchX = e.touches[0].clientX;
                const touchY = e.touches[0].clientY;
                
                // 移動を視覚的に表示
                currentElement.style.transform = 
                    `translate(${touchX - touchStartX}px, ${touchY - touchStartY}px)`;
            }
        });

        document.addEventListener('touchend', (e) => {
            if (currentElement) {
                currentElement.style.opacity = '1';
                currentElement.style.transform = '';
                
                // ドロップ位置を決定
                const dropTarget = document.elementFromPoint(
                    e.changedTouches[0].clientX,
                    e.changedTouches[0].clientY
                );
                
                // タイルの入れ替え処理
                if (dropTarget && dropTarget.classList.contains('stream-tile')) {
                    this.swapTiles(currentElement, dropTarget);
                }
                
                currentElement = null;
            }
        });

        // ピンチズーム無効化（アプリ内での誤操作防止）
        document.addEventListener('touchmove', (e) => {
            if (e.touches.length > 1) {
                e.preventDefault();
            }
        }, { passive: false });
    }

    optimizePerformance() {
        // モバイルでのストリーム品質を調整
        if (window.streamManager) {
            // 解像度を下げる
            window.streamManager.mobileMode = true;
            window.streamManager.maxResolution = { width: 640, height: 480 };
            
            // フレームレートを制限
            window.streamManager.targetFPS = 15;
            
            // 同時ストリーム数を制限
            window.streamManager.maxStreams = 4;
        }

        // PointCloudの点数を削減
        if (window.PointCloudViewer) {
            window.PointCloudViewer.maxPoints = 50000; // モバイルでは5万点まで
        }
    }

    adjustUI() {
        // モバイル用のCSSクラスを追加
        document.body.classList.add('mobile-device');
        
        // ビューポートメタタグを確認
        let viewport = document.querySelector('meta[name="viewport"]');
        if (!viewport) {
            viewport = document.createElement('meta');
            viewport.name = 'viewport';
            document.head.appendChild(viewport);
        }
        viewport.content = 'width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no';

        // モバイル用のスタイルを追加
        const style = document.createElement('style');
        style.textContent = `
            .mobile-device .header {
                padding: 0.5rem;
            }
            
            .mobile-device .layout-selector {
                display: flex;
                overflow-x: auto;
                -webkit-overflow-scrolling: touch;
            }
            
            .mobile-device .stream-tile {
                min-height: 150px;
            }
            
            .mobile-device .tile-controls {
                opacity: 1; /* 常に表示 */
            }
            
            .mobile-device .side-panel {
                width: 100%;
                right: -100%;
            }
            
            .mobile-device .stream-grid.layout-quad {
                grid-template-columns: 1fr;
                grid-template-rows: repeat(4, minmax(200px, 1fr));
            }
            
            /* スワイプメニュー */
            .mobile-device .quick-menu {
                position: fixed;
                bottom: 0;
                left: 0;
                right: 0;
                top: auto;
                border-radius: 16px 16px 0 0;
                padding-bottom: env(safe-area-inset-bottom);
            }
        `;
        document.head.appendChild(style);
    }

    swapTiles(tile1, tile2) {
        const parent = tile1.parentNode;
        const temp = document.createElement('div');
        
        parent.insertBefore(temp, tile1);
        parent.insertBefore(tile1, tile2);
        parent.insertBefore(tile2, temp);
        parent.removeChild(temp);
    }

    // ネットワーク状態の監視
    monitorNetworkStatus() {
        const updateNetworkStatus = () => {
            const status = navigator.onLine ? 'online' : 'offline';
            document.body.dataset.networkStatus = status;
            
            if (status === 'offline') {
                this.showNotification('ネットワーク接続が失われました', 'error');
            } else {
                this.showNotification('ネットワークに再接続しました', 'success');
            }
        };

        window.addEventListener('online', updateNetworkStatus);
        window.addEventListener('offline', updateNetworkStatus);

        // 接続速度の推定
        if ('connection' in navigator) {
            navigator.connection.addEventListener('change', () => {
                const connection = navigator.connection;
                console.log('Network type:', connection.effectiveType);
                console.log('Downlink:', connection.downlink, 'Mbps');
                
                // 低速接続の場合は品質を自動調整
                if (connection.effectiveType === '2g' || connection.effectiveType === 'slow-2g') {
                    this.setLowBandwidthMode();
                }
            });
        }
    }

    setLowBandwidthMode() {
        console.log('Low bandwidth detected, adjusting quality...');
        
        if (window.streamManager) {
            window.streamManager.lowBandwidthMode = true;
            // ストリーム品質をさらに下げる
            window.streamManager.maxResolution = { width: 320, height: 240 };
            window.streamManager.targetFPS = 10;
        }
        
        this.showNotification('低速接続を検出: 品質を調整しました', 'warning');
    }

    showNotification(message, type = 'info') {
        // 通知を表示
        const notification = document.createElement('div');
        notification.className = `mobile-notification ${type}`;
        notification.textContent = message;
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            left: 50%;
            transform: translateX(-50%);
            background: ${type === 'error' ? '#ef4444' : type === 'warning' ? '#f59e0b' : '#10b981'};
            color: white;
            padding: 12px 24px;
            border-radius: 8px;
            z-index: 1000;
            animation: slideDown 0.3s ease;
        `;
        
        document.body.appendChild(notification);
        
        setTimeout(() => {
            notification.remove();
        }, 3000);
    }
}

// 自動初期化
document.addEventListener('DOMContentLoaded', () => {
    const mobileConfig = new MobileConfig();
    
    // モバイル最適化を適用
    mobileConfig.applyMobileOptimizations();
    
    // ネットワーク監視開始
    mobileConfig.monitorNetworkStatus();
    
    // グローバルに公開
    window.mobileConfig = mobileConfig;
    
    // StreamManagerの設定を更新
    if (window.streamManager) {
        // ROSBridge URLを動的に設定
        window.streamManager.defaultROSBridgeURL = mobileConfig.getROSBridgeURL();
        
        console.log('Mobile Config Applied:', {
            isMobile: mobileConfig.isMobile,
            serverIP: mobileConfig.serverIP,
            rosbridgeURL: mobileConfig.getROSBridgeURL()
        });
    }
});

export default MobileConfig;