#pragma once

/// \file transaction.hpp
/// \brief Transaction — the implicit-animation scope (CATransaction, §9).
///
/// Attribute changes made while a Transaction is alive on the current thread
/// animate instead of snapping:
///
/// ```cpp
/// {
///     Transaction t(0.3f, Ease::InOut);
///     stage.find("hud")->opacity(0.0f).position(24, -120);  // both animate
/// }
/// stage.find("hud")->hidden(true);  // outside the scope: immediate
/// ```
///
/// Transactions nest; the **innermost** one decides the duration and easing
/// of the changes inside it (§15-6). The class is scope-bound RAII: create
/// it on the stack, never on the heap, and don't move it across threads —
/// the active stack is thread-local, matching the Stage's single-thread
/// ownership rule (§6.3).

#include "fluent_stage/types.hpp"

namespace fluent_stage {

class Transaction {
public:
    /// Opens an animation scope: attribute changes on this thread animate
    /// over \p duration seconds with \p ease until the scope closes.
    explicit Transaction(float duration, Ease ease = Ease::InOut)
        : duration_(duration), ease_(ease), prev_(head()) {
        head() = this;
    }

    ~Transaction() { head() = prev_; }

    Transaction(const Transaction&) = delete;
    Transaction& operator=(const Transaction&) = delete;

    /// Seconds over which changes in this scope animate.
    float duration() const { return duration_; }
    /// Easing curve for changes in this scope.
    Ease ease() const { return ease_; }

    /// The innermost active Transaction on this thread, or nullptr when
    /// attribute changes should apply immediately.
    static const Transaction* active() { return head(); }

private:
    static Transaction*& head() {
        thread_local Transaction* head_ptr = nullptr;
        return head_ptr;
    }

    float duration_;
    Ease ease_;
    Transaction* prev_;
};

}  // namespace fluent_stage
