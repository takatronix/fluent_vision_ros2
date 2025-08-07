// ROS2 PointCloud2 Viewer for FluentVision
import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.158.0/build/three.module.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.158.0/examples/jsm/controls/OrbitControls.js';

class PointCloudViewer {
    constructor(container) {
        this.container = container;
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.pointCloud = null;
        this.ros = null;
        this.topic = null;
        
        this.init();
    }

    init() {
        // Three.jsシーンのセットアップ
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x000000);
        
        // カメラ設定
        const width = this.container.clientWidth;
        const height = this.container.clientHeight;
        this.camera = new THREE.PerspectiveCamera(
            75, width / height, 0.1, 1000
        );
        this.camera.position.set(0, 2, 5);
        
        // レンダラー設定
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(width, height);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.container.appendChild(this.renderer.domElement);
        
        // コントロール設定（マウスで回転・ズーム）
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        
        // グリッドとAxesを追加
        this.addHelpers();
        
        // アニメーションループ開始
        this.animate();
        
        // リサイズ対応
        window.addEventListener('resize', () => this.onResize());
    }

    addHelpers() {
        // グリッド
        const gridHelper = new THREE.GridHelper(10, 10, 0x444444, 0x222222);
        this.scene.add(gridHelper);
        
        // 座標軸（X:赤, Y:緑, Z:青）
        const axesHelper = new THREE.AxesHelper(2);
        this.scene.add(axesHelper);
        
        // ライト
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        this.scene.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
        directionalLight.position.set(5, 5, 5);
        this.scene.add(directionalLight);
    }

    connectToROS(rosbridge_url = 'ws://localhost:9090') {
        if (typeof ROSLIB === 'undefined') {
            console.error('ROSLIB not loaded. Loading from CDN...');
            this.loadROSLIB().then(() => {
                this.connectToROS(rosbridge_url);
            });
            return;
        }

        this.ros = new ROSLIB.Ros({
            url: rosbridge_url
        });

        this.ros.on('connection', () => {
            console.log('Connected to ROS bridge for PointCloud');
            this.showStatus('ROS接続成功', 'success');
        });

        this.ros.on('error', (error) => {
            console.error('ROS connection error:', error);
            this.showStatus('ROS接続エラー', 'error');
        });

        this.ros.on('close', () => {
            console.log('ROS connection closed');
            this.showStatus('ROS切断', 'warning');
        });
    }

    subscribeToPointCloud(topicName = '/camera/depth/points') {
        if (!this.ros) {
            console.error('ROS not connected');
            return;
        }

        // 既存のトピックをunsubscribe
        if (this.topic) {
            this.topic.unsubscribe();
        }

        this.topic = new ROSLIB.Topic({
            ros: this.ros,
            name: topicName,
            messageType: 'sensor_msgs/PointCloud2',
            compression: 'cbor'  // 圧縮を使用してパフォーマンス向上
        });

        this.topic.subscribe((message) => {
            this.processPointCloud(message);
        });

        console.log(`Subscribed to PointCloud2 topic: ${topicName}`);
    }

    processPointCloud(msg) {
        // PointCloud2メッセージの解析
        const points = this.parsePointCloud2(msg);
        
        if (points.length === 0) {
            console.warn('No points in cloud');
            return;
        }

        // 既存のポイントクラウドを削除
        if (this.pointCloud) {
            this.scene.remove(this.pointCloud);
            this.pointCloud.geometry.dispose();
            this.pointCloud.material.dispose();
        }

        // 新しいポイントクラウドを作成
        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(points.length * 3);
        const colors = new Float32Array(points.length * 3);

        for (let i = 0; i < points.length; i++) {
            const point = points[i];
            positions[i * 3] = point.x;
            positions[i * 3 + 1] = point.y;
            positions[i * 3 + 2] = point.z;
            
            // 深度に基づいた色付け
            const depth = Math.sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            const hue = (1.0 - Math.min(depth / 5.0, 1.0)) * 0.7; // 近い=赤、遠い=青
            const color = new THREE.Color();
            color.setHSL(hue, 1.0, 0.5);
            
            colors[i * 3] = color.r;
            colors[i * 3 + 1] = color.g;
            colors[i * 3 + 2] = color.b;
        }

        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));

        const material = new THREE.PointsMaterial({
            size: 0.01,
            vertexColors: true,
            sizeAttenuation: true
        });

        this.pointCloud = new THREE.Points(geometry, material);
        this.scene.add(this.pointCloud);

        // ステータス更新
        this.showStatus(`Points: ${points.length}`, 'info');
    }

    parsePointCloud2(msg) {
        const points = [];
        
        // PointCloud2のデータ構造を解析
        const data = atob(msg.data); // Base64デコード
        const dataView = new DataView(new ArrayBuffer(data.length));
        
        for (let i = 0; i < data.length; i++) {
            dataView.setUint8(i, data.charCodeAt(i));
        }

        // フィールドオフセットを取得
        let xOffset = -1, yOffset = -1, zOffset = -1;
        let rgbOffset = -1;
        
        for (const field of msg.fields) {
            if (field.name === 'x') xOffset = field.offset;
            if (field.name === 'y') yOffset = field.offset;
            if (field.name === 'z') zOffset = field.offset;
            if (field.name === 'rgb') rgbOffset = field.offset;
        }

        if (xOffset === -1 || yOffset === -1 || zOffset === -1) {
            console.error('Required fields (x,y,z) not found in PointCloud2');
            return points;
        }

        // ポイントを抽出
        const pointStep = msg.point_step;
        const numPoints = Math.floor(dataView.byteLength / pointStep);
        
        for (let i = 0; i < numPoints && i < 100000; i++) { // 最大10万点に制限
            const offset = i * pointStep;
            
            const x = dataView.getFloat32(offset + xOffset, true);
            const y = dataView.getFloat32(offset + yOffset, true);
            const z = dataView.getFloat32(offset + zOffset, true);
            
            // 無効な点をフィルタ
            if (!isNaN(x) && !isNaN(y) && !isNaN(z) && 
                isFinite(x) && isFinite(y) && isFinite(z)) {
                points.push({ x, y, z });
            }
        }

        return points;
    }

    // デモ用のポイントクラウド生成
    generateDemoPointCloud() {
        const points = [];
        const numPoints = 10000;
        
        for (let i = 0; i < numPoints; i++) {
            // 球状のポイントクラウドを生成
            const theta = Math.random() * Math.PI * 2;
            const phi = Math.random() * Math.PI;
            const r = 1 + Math.random() * 0.5;
            
            points.push({
                x: r * Math.sin(phi) * Math.cos(theta),
                y: r * Math.sin(phi) * Math.sin(theta),
                z: r * Math.cos(phi)
            });
        }
        
        // ダミーメッセージとして処理
        this.processPointCloud({
            fields: [
                {name: 'x', offset: 0},
                {name: 'y', offset: 4},
                {name: 'z', offset: 8}
            ],
            point_step: 12,
            data: btoa('dummy') // ダミーデータ
        });
        
        // 実際には直接ポイントを処理
        this.createPointCloudFromPoints(points);
    }

    createPointCloudFromPoints(points) {
        if (this.pointCloud) {
            this.scene.remove(this.pointCloud);
        }

        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(points.length * 3);
        const colors = new Float32Array(points.length * 3);

        for (let i = 0; i < points.length; i++) {
            const point = points[i];
            positions[i * 3] = point.x;
            positions[i * 3 + 1] = point.y;
            positions[i * 3 + 2] = point.z;
            
            // カラフルな色付け
            const hue = (i / points.length) * 0.7;
            const color = new THREE.Color();
            color.setHSL(hue, 1.0, 0.5);
            
            colors[i * 3] = color.r;
            colors[i * 3 + 1] = color.g;
            colors[i * 3 + 2] = color.b;
        }

        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));

        const material = new THREE.PointsMaterial({
            size: 0.02,
            vertexColors: true,
            sizeAttenuation: true
        });

        this.pointCloud = new THREE.Points(geometry, material);
        this.scene.add(this.pointCloud);
    }

    animate() {
        requestAnimationFrame(() => this.animate());
        
        this.controls.update();
        
        // ポイントクラウドをゆっくり回転
        if (this.pointCloud) {
            this.pointCloud.rotation.y += 0.001;
        }
        
        this.renderer.render(this.scene, this.camera);
    }

    onResize() {
        const width = this.container.clientWidth;
        const height = this.container.clientHeight;
        
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        
        this.renderer.setSize(width, height);
    }

    showStatus(message, type = 'info') {
        // ステータス表示（実装は親コンポーネントに依存）
        console.log(`[PointCloud ${type}]: ${message}`);
    }

    async loadROSLIB() {
        return new Promise((resolve, reject) => {
            const script = document.createElement('script');
            script.src = 'https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js';
            script.onload = resolve;
            script.onerror = reject;
            document.head.appendChild(script);
        });
    }

    dispose() {
        if (this.topic) {
            this.topic.unsubscribe();
        }
        if (this.ros) {
            this.ros.close();
        }
        if (this.renderer) {
            this.renderer.dispose();
        }
        if (this.controls) {
            this.controls.dispose();
        }
    }
}

// StreamManagerに統合するための拡張
export function addPointCloudSupport(streamManager) {
    // ポイントクラウドタイプを追加
    const originalCreateStreamElement = streamManager.createStreamElement;
    
    streamManager.createStreamElement = function(container, config) {
        if (config.type === 'pointcloud') {
            // Three.jsキャンバスを作成
            const canvas = document.createElement('canvas');
            canvas.style.width = '100%';
            canvas.style.height = '100%';
            container.appendChild(canvas);
            
            // PointCloudViewerを初期化
            const viewer = new PointCloudViewer(container);
            
            // ROSに接続
            if (config.rosbridge) {
                viewer.connectToROS(config.rosbridge);
                viewer.subscribeToPointCloud(config.rosTopic || '/camera/depth/points');
            } else {
                // デモモード
                viewer.generateDemoPointCloud();
            }
            
            // viewerを保存
            container.pointCloudViewer = viewer;
        } else {
            originalCreateStreamElement.call(this, container, config);
        }
    };
}

// グローバルに公開
window.PointCloudViewer = PointCloudViewer;
window.addPointCloudSupport = addPointCloudSupport;