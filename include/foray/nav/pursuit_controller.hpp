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

#ifndef FORAY__NAV__PURSUIT_CONTROLLER_HPP_
#define FORAY__NAV__PURSUIT_CONTROLLER_HPP_

#include "foray/nav/controller.hpp"

namespace foray {
namespace nav {

/// 纯追踪（全向底盘）参数。
///
/// 默认值口径：幅值与周期取自参考实机配置，**我方车体未标定，一律待实测**
/// （见 docs/nav_architecture.md §11）。
struct PursuitParams {
    /// 前视距离（米）：速度自适应时为上限，固定时为定值。
    double lookahead_min{0.5};
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
    /// 估计曲率时向前/向后看的距离（米）。
    double curvature_forward_dist{0.7};

    /// 停止容差（米）：距终点小于它时输出零速度。
    /// @note **不是到达判定**——到达由状态机按 `NavGoal::xy_tolerance` 判。
    double stop_tolerance{0.15};
};

/// 纯追踪控制器：全向底盘，不做航向对齐，直接朝前视点平移。
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

    /// 最近一次输出的速度缩放因子（0..1，可观测用）。
    double last_scale() const {
        return last_scale_;
    }

  private:
    PursuitParams params_{};
    VelocityCommand last_command_{};
    double last_scale_{1.0};
    bool has_last_{false};
};

} // namespace nav
} // namespace foray

#endif // FORAY__NAV__PURSUIT_CONTROLLER_HPP_
