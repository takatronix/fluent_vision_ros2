#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <utility>
#include <functional>
#include "fluent_text.hpp"
#include "fluent_lib/ui/window_base.hpp"

namespace fluent_ui {

/**
 * AutoSizeInfoWindow
 * - テキスト行の内容に応じて自動リサイズする情報ウィンドウ
 * - 選択BBoxの左右に追従（バブル風にコールアウトラインを描画）
 * - WindowBaseのonDrawでテキストを描画し、renderをオーバーライドしてラインを追加
 */
class AutoSizeInfoWindow : public WindowBase {
public:
    enum class Side { Left, Right };

    // 自由テキスト（\n区切り対応）。auto_wrap=true で max_text_width_px_ に合わせて自動改行
    void setText(const std::string &text, bool auto_wrap=true) {
        text_raw_ = text; auto_wrap_ = auto_wrap; rebuildLines(); needs_layout_ = true;
    }
    void setLines(std::vector<std::string> lines) { lines_ = std::move(lines); needs_layout_ = true; }
    void setSide(Side s) { side_ = s; needs_layout_ = true; }
    void setAnchorBBox(const cv::Rect &bbox) { anchor_bbox_ = bbox; needs_layout_ = true; }
    void setGapPx(int gap) { gap_px_ = std::max(0, gap); needs_layout_ = true; }
    void setYOffsetPx(int yoff) { y_offset_px_ = yoff; needs_layout_ = true; }
    void setMaxTextWidthPx(int max_w) { max_text_width_px_ = std::max(64, max_w); needs_layout_ = true; }
    void setTextStyle(double scale, int thickness, cv::Scalar color_fg, cv::Scalar color_shadow) {
        font_scale_ = scale; font_thickness_ = thickness; fg_ = color_fg; shadow_ = color_shadow; needs_layout_ = true;
    }
    void setLineHeightPx(int px) { line_height_px_ = std::max(0, px); needs_layout_ = true; }
    void setUseFluentText(bool enable) { use_fluent_text_ = enable; }

    // 右ウィンドウ固定幅にしたい場合はここで強制幅を与える（0で無効）
    void setForcedWidthPx(int w) { forced_width_px_ = std::max(0, w); needs_layout_ = true; }

    // 完全固定: 座標補間やアンカー追従を無効化する
    void setLocked(bool locked) { locked_ = locked; }

    // コールアウト: 明示アンカーポイント（例: アプローチポイント）を設定
    void setCalloutAnchor(const cv::Point &p) { callout_anchor_point_ = p; callout_anchor_enabled_ = true; }
    void clearCalloutAnchor() { callout_anchor_enabled_ = false; }
    void setCalloutThickness(int px) { callout_thickness_ = std::max(1, px); }

    // tickでスムーズに追従（WindowBaseが補間）。ここでは必要に応じてレイアウト更新だけ行う。
    void tick(double dt_seconds) {
        if (needs_layout_) updateLayout();
        if (locked_) {
            // 目標値=現在値に固定して補間を抑止
            WindowBase::setImmediateRect(WindowBase::currentRect());
            // αも固定（visible状態を維持）
        } else {
            WindowBase::tick(dt_seconds);
        }
    }

    // コールアウトラインも描画するために render を拡張
    void render(cv::Mat &base) override {
        if (needs_layout_) updateLayout();
        // 先にラインを描く（αはWindow側で管理）
        if (alpha() > 0.01f && anchor_bbox_.width > 0 && anchor_bbox_.height > 0) {
            drawCallout(base);
        }
        // 本体の半透明ウィンドウを合成
        WindowBase::render(base);
    }

protected:
    void onDraw(cv::Mat &canvas) override {
        int pad = padding_px_;
        int lh  = line_height_px_ > 0 ? line_height_px_ : static_cast<int>(std::round(20 * font_scale_));
        int y = pad + lh; // 初期ベースライン
        for (const auto &s : lines_) {
            // 文字化け回避: 影色をやや薄くし、描画座標を整数に丸め
            cv::Point org(pad, y);
            if (use_fluent_text_) fluent::text::drawShadow(canvas, s, org, fg_, cv::Scalar(32,32,32), font_scale_, font_thickness_, 0);
            else drawShadowText(canvas, s, org, fg_, cv::Scalar(32,32,32), font_scale_, font_thickness_);
            y += lh;
        }
    }

private:
    void rebuildLines() {
        if (text_raw_.empty()) { lines_.clear(); return; }
        // まず \n で分割
        std::vector<std::string> raw_lines; raw_lines.reserve(8);
        {
            size_t start = 0; size_t pos = 0;
            while ((pos = text_raw_.find('\n', start)) != std::string::npos) {
                raw_lines.emplace_back(text_raw_.substr(start, pos - start));
                start = pos + 1;
            }
            raw_lines.emplace_back(text_raw_.substr(start));
        }
        // 自動改行しない場合はそのまま
        if (!auto_wrap_) { lines_ = std::move(raw_lines); return; }
        // 自動改行: 文字単位でgetTextSizeしながら積み上げ（日本語/空白非依存の簡易実装）
        std::vector<std::string> wrapped; wrapped.reserve(raw_lines.size());
        for (const auto &line : raw_lines) {
            std::string cur;
            for (size_t i = 0; i < line.size(); ) {
                // UTF-8の可変長を簡易処理: 1バイトずつ（厳密ではないが描画と概ね整合）
                cur.push_back(line[i]);
                ++i;
                // 行幅を測定
                int base = 0; cv::Size sz = cv::getTextSize(cur, cv::FONT_HERSHEY_SIMPLEX, font_scale_, font_thickness_, &base);
                if (sz.width > max_text_width_px_ - padding_px_ * 2 && cur.size() > 1) {
                    // 直前までを確定
                    // 最後の1文字を次行へ回す
                    char last = cur.back(); cur.pop_back();
                    wrapped.push_back(cur);
                    cur.clear(); cur.push_back(last);
                }
            }
            wrapped.push_back(cur);
        }
        lines_ = std::move(wrapped);
    }

    void updateLayout() {
        needs_layout_ = false;
        // テキスト→行再構成（フォントや幅が変わった可能性もある）
        if (!text_raw_.empty()) rebuildLines();
        // テキストから必要サイズを計算
        int maxw = 0; int totalh = 0; int base = 0;
        int lh  = line_height_px_ > 0 ? line_height_px_ : static_cast<int>(std::round(20 * font_scale_));
        for (const auto &s : lines_) {
            int w = 0;
            if (use_fluent_text_) {
                w = measureFluentLineWidthPx(s);
            } else {
                cv::Size sz = cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, font_scale_, font_thickness_, &base);
                w = sz.width;
            }
            maxw = std::max(maxw, w);
            totalh += lh;
        }
        // 強制幅 or 最大テキスト幅 + パディング
        int inner_w = forced_width_px_ > 0 ? forced_width_px_ - padding_px_ * 2 : std::min(maxw, max_text_width_px_);
        int W = std::max(64, inner_w + padding_px_ * 2);
        int H = std::max(32, totalh + padding_px_ * 2);

        // アンカーから配置
        cv::Rect r;
        if (side_ == Side::Left) {
            r.x = anchor_bbox_.x - gap_px_ - W;
        } else {
            r.x = anchor_bbox_.x + anchor_bbox_.width + gap_px_;
        }
        r.y = anchor_bbox_.y + y_offset_px_;
        r.width = W; r.height = H;
        setTargetRect(r);
        // 初期化時に瞬時反映したい場合は setImmediateRect を呼ぶ側で行う
    }

    void drawCallout(cv::Mat &base) {
        cv::Rect win = currentRect();
        if (win.width <= 0 || win.height <= 0) return;
        // コールアウトの起点（アプローチ点があれば優先。なければアンカーBBoxの側面中心）
        cv::Point p_anchor;
        if (callout_anchor_enabled_) {
            p_anchor = callout_anchor_point_;
        } else {
            p_anchor = cv::Point(side_ == Side::Left ? anchor_bbox_.x : anchor_bbox_.x + anchor_bbox_.width,
                                 std::clamp(anchor_bbox_.y + anchor_bbox_.height/2, 0, base.rows-1));
        }
        p_anchor.x = std::clamp(p_anchor.x, 0, std::max(0, base.cols-1));
        p_anchor.y = std::clamp(p_anchor.y, 0, std::max(0, base.rows-1));
        // ウィンドウ側の接続点（側面中央）
        cv::Point p_win(side_ == Side::Left ? (win.x + win.width) : win.x,
                        std::clamp(win.y + win.height/2, 0, base.rows-1));
        // 直線で接続（1px指定可能）
        cv::line(base, p_anchor, p_win, callout_color_, callout_thickness_, cv::LINE_AA);
    }

private:
    // 日本語レンダラで1行幅を推定（描画→非ゼロ矩形の幅）
    int measureFluentLineWidthPx(const std::string &s) const {
        const int test_w = std::max(512, max_text_width_px_ * 2);
        const int test_h = std::max(32, static_cast<int>(std::round(28 * font_scale_)) + font_thickness_ * 4);
        cv::Mat tmp(test_h, test_w, CV_8UC3, cv::Scalar(0,0,0));
        int baseline = static_cast<int>(std::round(20 * font_scale_));
        fluent::text::drawShadow(tmp, s, {padding_px_, baseline}, fg_, shadow_, font_scale_, font_thickness_, 0);
        cv::Mat gray; cv::cvtColor(tmp, gray, cv::COLOR_BGR2GRAY);
        cv::Mat mask; cv::threshold(gray, mask, 1, 255, cv::THRESH_BINARY);
        std::vector<cv::Point> nz; cv::findNonZero(mask, nz);
        if (nz.empty()) {
            int base = 0; return cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, font_scale_, font_thickness_, &base).width;
        }
        cv::Rect bb = cv::boundingRect(nz);
        return bb.width;
    }
    // 内容
    std::vector<std::string> lines_;
    std::string text_raw_;
    bool auto_wrap_ {true};
    // レイアウト・スタイル
    int padding_px_ {8};
    int line_height_px_ {0}; // 0=auto(20*scale)
    double font_scale_ {0.7};
    int font_thickness_ {2};
    cv::Scalar fg_ {255,255,255};
    cv::Scalar shadow_ {0,0,0};
    int max_text_width_px_ {380};
    int forced_width_px_ {0};
    bool use_fluent_text_ {true};
    // アンカー
    cv::Rect anchor_bbox_ {0,0,0,0};
    Side side_ {Side::Right};
    int gap_px_ {8};
    int y_offset_px_ {0};
    // コールアウト
    cv::Scalar callout_color_ {200,200,200};
    int callout_thickness_ {1};
    bool callout_anchor_enabled_ {false};
    cv::Point callout_anchor_point_ {0,0};
    // 更新フラグ
    bool needs_layout_ {true};
    bool locked_ {false};
};

/**
 * MirrorBBoxWindow
 * - 左側用：選択枠と同じサイズで追従するウィンドウ（内容は自由）
 */
class MirrorBBoxWindow : public WindowBase {
public:
    void setAnchorBBox(const cv::Rect &bbox, int inset_px = 0) {
        cv::Rect r = bbox;
        if (inset_px > 0) {
            r.x += inset_px; r.y += inset_px;
            r.width = std::max(1, r.width - inset_px*2);
            r.height = std::max(1, r.height - inset_px*2);
        }
        setTargetRect(r);
    }
    void setContent(std::function<void(cv::Mat&)> drawer) { drawer_ = std::move(drawer); }
protected:
    void onDraw(cv::Mat &canvas) override {
        if (drawer_) drawer_(canvas);
    }
private:
    std::function<void(cv::Mat&)> drawer_;
};

} // namespace fluent_ui


