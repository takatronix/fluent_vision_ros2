# 🏗️ FluentVision ROS2 - プロジェクト構造理解ガイド

## 🎯 AIのためのプロジェクト構造理解システム

### 作業開始時の必須チェックリスト
AIは作業開始前に以下を**必ず実行**し、理解した内容をユーザーに報告する：

1. **📋 プロジェクト概要の確認**
   ```bash
   # 1. メインREADMEを読む
   cat README.md
   
   # 2. 全パッケージ構成を把握
   find src/ -name "package.xml" | head -10
   
   # 3. 最新のTODOを確認
   find . -name "TODO*.md" -o -name "TODO*.txt" | xargs cat
   ```

2. **🗂️ アーキテクチャ理解**
   ```bash
   # 依存関係の把握
   grep -r "depend>" src/*/package.xml | sort
   
   # FluentLibの構造確認
   ls -la src/common/fluent_lib/include/
   cat src/common/fluent_lib/include/fluent.hpp
   ```

3. **🔄 記憶再構築報告**
   - プロジェクトの6つのコア価値を復唱
   - 主要パッケージの役割を要約
   - 現在の開発段階を確認
   - 既知の問題・制約を列挙

## 🧠 抽象思考促進フレームワーク

### レイヤー別理解モデル
```
📊 抽象度レベル（上から下へ）
├─ Level 6: ビジネス価値 (便利・楽・簡単・スムース・高速・美しい)
├─ Level 5: アーキテクチャ (AI・センサー・ストリーミング・共通)
├─ Level 4: パッケージ (fv_aspara_analyzer, fv_realsense, etc.)
├─ Level 3: クラス設計 (抽象基底クラス → 具象実装)
├─ Level 2: 関数・メソッド (Fluent API チェーン)
└─ Level 1: 実装詳細 (C++コード)
```

### 抽象クラス設計の必須思考プロセス

#### 🎯 新機能追加時の思考ステップ
1. **業務価値の確認**: 6つの価値のどれに貢献するか？
2. **類似機能の調査**: 既存ライブラリに同様の機能はないか？
3. **抽象化の検討**: 
   - この機能は将来どう拡張される？
   - 他の用途にも使える抽象基底クラスが必要？
   - インターフェースはどう設計すべき？
4. **Fluentチェーンの検討**: メソッドチェーンに組み込めるか？
5. **再利用性の評価**: 他のパッケージでも使える設計か？

#### 📐 抽象クラス設計テンプレート
```cpp
// ❌ 悪い例: 具体的すぎる
class AsparagusCutter { 
    void cutAsparagus(); 
};

// ✅ 良い例: 抽象的で拡張可能
class VegetableTool {
public:
    virtual ~VegetableTool() = default;
    virtual bool process(const Vegetable& item) = 0;
    virtual ToolStatus getStatus() const = 0;
};

class AsparagusCutter : public VegetableTool {
public:
    bool process(const Vegetable& item) override;
    ToolStatus getStatus() const override;
};
```

## 🗺️ パッケージ間依存関係マップ

### コア依存関係
```
fluent_lib (共通基盤)
    ↑
    ├── fv_realsense (センサー)
    ├── fv_object_detector (AI検出)
    ├── fv_aspara_analyzer (AI分析) 
    └── fv_recorder (ストリーミング)
    
データフロー:
センサー → AI処理 → ストリーミング配信
```

### Fluent APIの統合ポイント
```cpp
// 理想的なFluent統合例
auto result = FluentVision::from(camera_image)
    .detectObjects()      // fv_object_detector
    .analyzeAsparagus()   // fv_aspara_analyzer  
    .stream()             // fv_recorder
    .toMulticast();       // fv_image_distributor
```

## 🔍 AIが見落としがちなポイント

### 1. 横断的関心事の見逃し
- **ログ出力**: 全パッケージで統一された形式？
- **エラー処理**: 共通の例外クラス使用？
- **設定管理**: パラメータ読み込みの統一？
- **メトリクス**: 性能測定の標準化？

### 2. 将来拡張性の考慮不足
- **新しいカメラ追加**: センサー抽象化は十分？
- **新しいAIモデル**: 推論エンジンの抽象化は？
- **新しい配信先**: ストリーミング基盤の柔軟性は？

### 3. パフォーマンス設計
- **メモリ効率**: 不要なコピーを避ける設計？
- **リアルタイム性**: フレームドロップ防止策？
- **CPU使用率**: 並列処理の最適化？

## 📚 設計パターン適用ガイド

### よく使うパターンと適用場面

#### 1. Factory Pattern
```cpp
// センサー作成時
std::unique_ptr<Sensor> SensorFactory::create(SensorType type);
```

#### 2. Strategy Pattern  
```cpp
// AI推論エンジン切り替え
class InferenceStrategy {
public:
    virtual ~InferenceStrategy() = default;
    virtual Result infer(const Image& input) = 0;
};
```

#### 3. Observer Pattern
```cpp
// 検出結果の通知
class DetectionObserver {
public:
    virtual ~DetectionObserver() = default;
    virtual void onDetection(const DetectionResult& result) = 0;
};
```

#### 4. Builder Pattern
```cpp
// 複雑な設定オブジェクト構築
auto config = ConfigBuilder()
    .camera(CameraType::RealSense)
    .resolution(1920, 1080)
    .aiModel("yolov10n.onnx")
    .streaming(true)
    .build();
```

## 🚨 アンチパターン警告

### AIが陥りやすい悪い設計
1. **Godクラス**: 1つのクラスに機能を詰め込みすぎ
2. **具体実装の直接依存**: インターフェースを使わない
3. **責任の混在**: 1つのクラスで複数の責任を持つ
4. **例外の無視**: エラー処理の軽視
5. **テストしにくい設計**: 依存注入未対応

### 設計品質チェックリスト
- [ ] SOLID原則に準拠している？
- [ ] 抽象基底クラスを適切に使用？
- [ ] Fluent APIに統合可能？
- [ ] 他のパッケージで再利用可能？
- [ ] テストが書きやすい構造？
- [ ] 将来の要求変更に柔軟？

---

**🌟 記憶すべき核心**: FluentVisionは「Flow Like Water」の哲学で、思考がそのままコードに表現される美しいシステム。常に抽象化レベルを意識し、将来の拡張性と現在の実用性のバランスを保つ。
