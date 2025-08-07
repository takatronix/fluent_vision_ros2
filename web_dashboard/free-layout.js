// Free Layout Mode for FluentVision
// 完全に自由な配置・リサイズ・重ね順制御

class FreeLayoutManager {
    constructor() {
        this.tiles = new Map();
        this.activeZ = 1000;
        this.draggedElement = null;
        this.resizingElement = null;
        this.dragOffset = { x: 0, y: 0 };
        this.minSize = { width: 200, height: 150 };
        
        this.init();
    }

    init() {
        this.setupEventListeners();
        this.loadSavedPositions();
    }

    enableFreeLayout() {
        const grid = document.getElementById('streamGrid');
        grid.classList.add('free-layout-mode');
        
        // すべてのタイルを自由配置可能にする
        document.querySelectorAll('.stream-tile').forEach(tile => {
            this.makeFreeTile(tile);
        });
    }

    disableFreeLayout() {
        const grid = document.getElementById('streamGrid');
        grid.classList.remove('free-layout-mode');
        
        // タイルの位置をリセット
        document.querySelectorAll('.stream-tile').forEach(tile => {
            tile.style.position = '';
            tile.style.left = '';
            tile.style.top = '';
            tile.style.width = '';
            tile.style.height = '';
            tile.style.zIndex = '';
            tile.classList.remove('free-tile');
        });
    }

    makeFreeTile(tile) {
        if (tile.classList.contains('free-tile')) return;
        
        tile.classList.add('free-tile');
        
        // デフォルト位置とサイズを設定
        const rect = tile.getBoundingClientRect();
        const container = document.getElementById('streamGrid').getBoundingClientRect();
        
        const tileData = {
            id: tile.id,
            x: rect.left - container.left,
            y: rect.top - container.top,
            width: rect.width,
            height: rect.height,
            zIndex: this.activeZ++
        };
        
        // 保存された位置があれば使用
        const saved = this.tiles.get(tile.id);
        if (saved) {
            Object.assign(tileData, saved);
        }
        
        this.tiles.set(tile.id, tileData);
        this.applyPosition(tile, tileData);
        
        // ドラッグハンドルを追加
        this.addDragHandle(tile);
        
        // リサイズハンドルを追加
        this.addResizeHandles(tile);
        
        // クリックで最前面に
        tile.addEventListener('mousedown', (e) => {
            if (!e.target.classList.contains('resize-handle')) {
                this.bringToFront(tile);
            }
        });
    }

    addDragHandle(tile) {
        // ヘッダー部分をドラッグ可能にする
        const header = tile.querySelector('.tile-header');
        if (!header) return;
        
        header.style.cursor = 'move';
        
        header.addEventListener('mousedown', (e) => {
            if (e.target.closest('.tile-controls')) return; // ボタンは除外
            
            this.startDrag(tile, e);
            e.preventDefault();
        });
    }

    addResizeHandles(tile) {
        // 8方向のリサイズハンドルを追加
        const handles = ['n', 'ne', 'e', 'se', 's', 'sw', 'w', 'nw'];
        
        handles.forEach(direction => {
            const handle = document.createElement('div');
            handle.className = `resize-handle resize-${direction}`;
            handle.dataset.direction = direction;
            tile.appendChild(handle);
            
            handle.addEventListener('mousedown', (e) => {
                this.startResize(tile, direction, e);
                e.stopPropagation();
                e.preventDefault();
            });
        });
    }

    startDrag(tile, e) {
        this.draggedElement = tile;
        const rect = tile.getBoundingClientRect();
        const container = document.getElementById('streamGrid').getBoundingClientRect();
        
        this.dragOffset = {
            x: e.clientX - rect.left,
            y: e.clientY - rect.top
        };
        
        tile.classList.add('dragging');
        this.bringToFront(tile);
        
        // グローバルイベントリスナー
        document.addEventListener('mousemove', this.handleDrag);
        document.addEventListener('mouseup', this.endDrag);
    }

    handleDrag = (e) => {
        if (!this.draggedElement) return;
        
        const container = document.getElementById('streamGrid').getBoundingClientRect();
        const tileData = this.tiles.get(this.draggedElement.id);
        
        // 新しい位置を計算
        let newX = e.clientX - container.left - this.dragOffset.x;
        let newY = e.clientY - container.top - this.dragOffset.y;
        
        // 境界チェック
        newX = Math.max(0, Math.min(newX, container.width - tileData.width));
        newY = Math.max(0, Math.min(newY, container.height - tileData.height));
        
        // 位置を更新
        tileData.x = newX;
        tileData.y = newY;
        
        this.applyPosition(this.draggedElement, tileData);
        
        // スナップ機能（オプション）
        if (e.shiftKey) {
            this.snapToGrid(this.draggedElement, tileData);
        }
    }

    endDrag = () => {
        if (this.draggedElement) {
            this.draggedElement.classList.remove('dragging');
            this.savePositions();
        }
        
        this.draggedElement = null;
        document.removeEventListener('mousemove', this.handleDrag);
        document.removeEventListener('mouseup', this.endDrag);
    }

    startResize(tile, direction, e) {
        this.resizingElement = tile;
        this.resizeDirection = direction;
        this.resizeStart = {
            x: e.clientX,
            y: e.clientY,
            tileData: { ...this.tiles.get(tile.id) }
        };
        
        tile.classList.add('resizing');
        
        document.addEventListener('mousemove', this.handleResize);
        document.addEventListener('mouseup', this.endResize);
    }

    handleResize = (e) => {
        if (!this.resizingElement) return;
        
        const tileData = this.tiles.get(this.resizingElement.id);
        const start = this.resizeStart;
        const dx = e.clientX - start.x;
        const dy = e.clientY - start.y;
        const dir = this.resizeDirection;
        
        // 方向に応じてサイズと位置を調整
        if (dir.includes('e')) {
            tileData.width = Math.max(this.minSize.width, start.tileData.width + dx);
        }
        if (dir.includes('w')) {
            const newWidth = Math.max(this.minSize.width, start.tileData.width - dx);
            tileData.x = start.tileData.x + (start.tileData.width - newWidth);
            tileData.width = newWidth;
        }
        if (dir.includes('s')) {
            tileData.height = Math.max(this.minSize.height, start.tileData.height + dy);
        }
        if (dir.includes('n')) {
            const newHeight = Math.max(this.minSize.height, start.tileData.height - dy);
            tileData.y = start.tileData.y + (start.tileData.height - newHeight);
            tileData.height = newHeight;
        }
        
        this.applyPosition(this.resizingElement, tileData);
    }

    endResize = () => {
        if (this.resizingElement) {
            this.resizingElement.classList.remove('resizing');
            this.savePositions();
        }
        
        this.resizingElement = null;
        document.removeEventListener('mousemove', this.handleResize);
        document.removeEventListener('mouseup', this.endResize);
    }

    bringToFront(tile) {
        const tileData = this.tiles.get(tile.id);
        if (tileData) {
            tileData.zIndex = this.activeZ++;
            tile.style.zIndex = tileData.zIndex;
        }
    }

    snapToGrid(tile, tileData, gridSize = 20) {
        tileData.x = Math.round(tileData.x / gridSize) * gridSize;
        tileData.y = Math.round(tileData.y / gridSize) * gridSize;
        this.applyPosition(tile, tileData);
    }

    applyPosition(tile, data) {
        tile.style.position = 'absolute';
        tile.style.left = `${data.x}px`;
        tile.style.top = `${data.y}px`;
        tile.style.width = `${data.width}px`;
        tile.style.height = `${data.height}px`;
        tile.style.zIndex = data.zIndex;
    }

    savePositions() {
        const positions = {};
        this.tiles.forEach((data, id) => {
            positions[id] = data;
        });
        localStorage.setItem('fluentvision-free-layout', JSON.stringify(positions));
    }

    loadSavedPositions() {
        const saved = localStorage.getItem('fluentvision-free-layout');
        if (saved) {
            try {
                const positions = JSON.parse(saved);
                Object.entries(positions).forEach(([id, data]) => {
                    this.tiles.set(id, data);
                });
            } catch (e) {
                console.error('Failed to load saved positions:', e);
            }
        }
    }

    // プリセットレイアウト
    applyPreset(preset) {
        const container = document.getElementById('streamGrid').getBoundingClientRect();
        const tiles = Array.from(document.querySelectorAll('.stream-tile'));
        
        switch(preset) {
            case 'cascade':
                // ウィンドウズスタイルのカスケード配置
                tiles.forEach((tile, i) => {
                    const data = {
                        x: 20 + i * 30,
                        y: 20 + i * 30,
                        width: 400,
                        height: 300,
                        zIndex: 1000 + i
                    };
                    this.tiles.set(tile.id, data);
                    this.applyPosition(tile, data);
                });
                break;
                
            case 'mosaic':
                // タイル状配置（重ならない）
                const cols = Math.ceil(Math.sqrt(tiles.length));
                const tileWidth = container.width / cols;
                const tileHeight = container.height / Math.ceil(tiles.length / cols);
                
                tiles.forEach((tile, i) => {
                    const col = i % cols;
                    const row = Math.floor(i / cols);
                    const data = {
                        x: col * tileWidth,
                        y: row * tileHeight,
                        width: tileWidth - 10,
                        height: tileHeight - 10,
                        zIndex: 1000
                    };
                    this.tiles.set(tile.id, data);
                    this.applyPosition(tile, data);
                });
                break;
                
            case 'focus':
                // 1つを大きく、他を小さく周りに配置
                if (tiles.length > 0) {
                    // メインタイル
                    const mainData = {
                        x: container.width * 0.2,
                        y: container.height * 0.1,
                        width: container.width * 0.6,
                        height: container.height * 0.6,
                        zIndex: 2000
                    };
                    this.tiles.set(tiles[0].id, mainData);
                    this.applyPosition(tiles[0], mainData);
                    
                    // サブタイル
                    const subCount = tiles.length - 1;
                    tiles.slice(1).forEach((tile, i) => {
                        const angle = (i / subCount) * Math.PI * 2;
                        const radius = Math.min(container.width, container.height) * 0.35;
                        const data = {
                            x: container.width / 2 + Math.cos(angle) * radius - 100,
                            y: container.height / 2 + Math.sin(angle) * radius - 75,
                            width: 200,
                            height: 150,
                            zIndex: 1000 + i
                        };
                        this.tiles.set(tile.id, data);
                        this.applyPosition(tile, data);
                    });
                }
                break;
        }
        
        this.savePositions();
    }

    setupEventListeners() {
        // キーボードショートカット
        document.addEventListener('keydown', (e) => {
            if (e.ctrlKey || e.metaKey) {
                switch(e.key) {
                    case 'l': // Ctrl+L: 自由レイアウト切り替え
                        this.toggleFreeLayout();
                        e.preventDefault();
                        break;
                    case '1': // Ctrl+1: カスケード
                        this.applyPreset('cascade');
                        e.preventDefault();
                        break;
                    case '2': // Ctrl+2: モザイク
                        this.applyPreset('mosaic');
                        e.preventDefault();
                        break;
                    case '3': // Ctrl+3: フォーカス
                        this.applyPreset('focus');
                        e.preventDefault();
                        break;
                }
            }
        });
    }

    toggleFreeLayout() {
        const grid = document.getElementById('streamGrid');
        if (grid.classList.contains('free-layout-mode')) {
            this.disableFreeLayout();
        } else {
            this.enableFreeLayout();
        }
    }
}

// CSS追加
const style = document.createElement('style');
style.textContent = `
    /* 自由レイアウトモード */
    .free-layout-mode {
        position: relative !important;
        display: block !important;
        height: 100% !important;
    }
    
    .free-tile {
        position: absolute !important;
        cursor: default;
        transition: box-shadow 0.2s;
    }
    
    .free-tile:hover {
        box-shadow: 0 8px 32px rgba(99, 102, 241, 0.3);
    }
    
    .free-tile.dragging {
        opacity: 0.9;
        box-shadow: 0 16px 48px rgba(0, 0, 0, 0.4);
        cursor: move !important;
    }
    
    .free-tile.resizing {
        opacity: 0.9;
    }
    
    /* リサイズハンドル */
    .resize-handle {
        position: absolute;
        background: transparent;
        z-index: 10;
    }
    
    .resize-handle:hover {
        background: rgba(99, 102, 241, 0.5);
    }
    
    .resize-n, .resize-s {
        left: 10px;
        right: 10px;
        height: 6px;
        cursor: ns-resize;
    }
    
    .resize-n { top: 0; }
    .resize-s { bottom: 0; }
    
    .resize-e, .resize-w {
        top: 10px;
        bottom: 10px;
        width: 6px;
        cursor: ew-resize;
    }
    
    .resize-e { right: 0; }
    .resize-w { left: 0; }
    
    .resize-nw, .resize-ne, .resize-sw, .resize-se {
        width: 12px;
        height: 12px;
    }
    
    .resize-nw {
        top: 0;
        left: 0;
        cursor: nwse-resize;
    }
    
    .resize-ne {
        top: 0;
        right: 0;
        cursor: nesw-resize;
    }
    
    .resize-sw {
        bottom: 0;
        left: 0;
        cursor: nesw-resize;
    }
    
    .resize-se {
        bottom: 0;
        right: 0;
        cursor: nwse-resize;
    }
    
    /* レイアウトプリセットボタン */
    .layout-presets {
        display: flex;
        gap: 0.5rem;
        margin-left: 1rem;
    }
    
    .preset-btn {
        padding: 0.5rem;
        background: var(--bg-tertiary);
        border: 1px solid var(--border-color);
        border-radius: 4px;
        color: var(--text-secondary);
        cursor: pointer;
        font-size: 0.875rem;
        transition: all 0.2s;
    }
    
    .preset-btn:hover {
        background: var(--primary-color);
        color: white;
    }
    
    .preset-btn.active {
        background: var(--primary-color);
        color: white;
    }
`;
document.head.appendChild(style);

// エクスポート
window.FreeLayoutManager = FreeLayoutManager;