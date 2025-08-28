#pragma once
#include "hud_base.hpp"
#include "hud_widgets.hpp"

namespace fluent_ui {

// 将来的なHUDコンテナ（最小実装）
class HUD {
public:
    void add(std::shared_ptr<HUDWidget> w) { widgets_.push_back(std::move(w)); }
    void renderAll(cv::Mat& image) { for (auto& w : widgets_) if (w) w->render(image); }
private:
    std::vector<std::shared_ptr<HUDWidget>> widgets_;
};

} // namespace fluent_ui


