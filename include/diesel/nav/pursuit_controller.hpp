// Copyright 2026 Foray
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DIESEL__NAV__PURSUIT_CONTROLLER_HPP_
#define DIESEL__NAV__PURSUIT_CONTROLLER_HPP_

#include "diesel/nav/controller.hpp"

namespace diesel {
namespace nav {

/// 纯追踪（全向底盘）参数。
///
/// 默认值口径：幅值与周期取自参考实机配置，**我方车体未标定，一律待实测**
/// （见 docs/nav_architecture.md §11）。
struct PursuitParams {
    /// 前视距离（米）：速度自适应时为上限，固定时为定值。
    double lookahead_min{0.35};
    double lookahead_max{2.0};
    /// 速度缩放的前视时间（秒）：前视 = clamp(v * t, min, max)。
    double lookahead_time{1.0};

    /// 平移速度上限（绝对值，两个轴同用）。
    double max_linear{2.5};
    /// 速度变化率上限（m/s²），用于限幅，防甩。
    double max_accel{6.0};

    /// 到点减速：距终点该距离内开始线性降速。
    double approach_scaling_dist{1.0};
    /// 到点减速时的最低速度（米/秒）。
    double min_approach_speed{0.3};

    /// 曲率降速：曲率超过 min 开始降，超过 max 降到 reduction_ratio。
    double curvature_min{0.4};
    double curvature_max{0.7};
    double reduction_ratio_at_high_curvature{0.5};
    /// 拐角处的前视缩短比例：曲率到 `curvature_max` 时，前视缩到这个倍数。
    ///
    /// 为什么需要：前视越长，拐弯越贴内圈。2026-11-10 实测（**当时按 0.25 m 安全线**）：
    /// 纯追踪在左高地的两个角上切成 0.166 m，跟踪误差最大 0.241 m——比车体半宽还小。
    /// 缩到 0.35 倍后同一批任务跟踪误差 0.241 → 0.173 m。
    /// **注意**：它改善的是「跟随精度」，不直接提高「离墙距离」——后者靠 `robot_radius`
    /// 与 `inflation_span`。按实测车体 0.32 m 重跑，十组最大跟踪误差 0.185 m。
    double lookahead_corner_ratio{0.35};
    /// 估计曲率时向前/向后看的距离（米）。
    double curvature_forward_dist{0.7};

    /// 停止容差（米）：距终点小于它时输出零速度。
    /// @note **不是到达判定**——到达由状态机按 `NavGoal::xy_tolerance` 判。
    double stop_tolerance{0.15};

    /// 到点后要不要把车头对到路径末端的朝向。
    /// 全向底盘默认 false（平移就够用）；要「到位后朝向也对」的场景再打开。
    /// @note 2026-11-03 审计 PC-4：文档提过 `omega` 与 `require_heading`，代码里却是零。
    ///       现在补上：打开后到点阶段输出 omega 而不是硬停。
    bool align_heading_at_goal{false};
    /// 到点对朝向时的角度增益（1/s）与角速度上限（rad/s）。
    double heading_gain{2.0};
    double max_omega{2.0};
    /// 朝向误差小于该值（弧度）算对齐。
    double heading_tolerance{0.05};
};

/// 纯追踪控制器：全向底盘，**默认**不做航向对齐，直接朝前视点平移。
/// 需要「到位后朝向也对齐」时把 `PursuitParams::align_heading_at_goal` 打开。
///
/// 契约（见 controller.hpp）：
/// - 有状态：记住上一次指令做变化率限幅，`reset()` 归零；
/// - 输出前一定限幅：幅值、变化率都限；
/// - 路径为空、`dt` 非正、位姿非法时返回 `kInvalidInput`，不动上次指令。
class PursuitController : public LocalController {
  public:
    PursuitController() = default;
    explicit PursuitController(const PursuitParams& params) : params_(params) {
    }

    NavError compute(const Pose2& pose, const Path& path, double dt, VelocityCommand* out) override;

    void reset() override;

    double stop_tolerance() const override {
        return params_.stop_tolerance;
    }

    /// 当前参数（调参与测试用）。
    const PursuitParams& params() const {
        return params_;
    }

    /// 运行时缩短前视距离的比例（0.2~1.0）。
    /// 用途：前方被挡时先缩短前视，让车"贴着拐角"走小弧线，而不是对着远处的点硬切角。
    void set_lookahead_scale(double scale);

    /// 最近一次输出的速度缩放因子（0..1，可观测用）。
    double last_scale() const {
        return last_scale_;
    }

  private:
    PursuitParams params_{};
    double lookahead_scale_{1.0};
    VelocityCommand last_command_{};
    double last_scale_{1.0};
    bool has_last_{false};
};

} // namespace nav
} // namespace diesel

#endif // DIESEL__NAV__PURSUIT_CONTROLLER_HPP_
