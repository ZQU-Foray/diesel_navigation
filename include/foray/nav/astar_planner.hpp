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

#ifndef FORAY__NAV__ASTAR_PLANNER_HPP_
#define FORAY__NAV__ASTAR_PLANNER_HPP_

#include <vector>

#include "foray/nav/planner.hpp"

namespace foray {
namespace nav {

/// A* 全局规划器参数。
struct AstarParams {
    /// 膨胀代价折算成行走代价的权重：代价 252 的格子视为 weigh 的 (1 + weight) 倍。
    double cost_weight{3.0};
    /// 偏好区（kCostPreferred）的行走权重：小于 1 表示"更想走这里"。
    double prefer_weight{0.5};
    /// 是否做视线捷径化（去掉"能直连却还绕一格"的锯齿点）。
    bool smooth{true};
    /// 单次规划最多展开的格子数上限（防御性上界）。可被 `PlanningOptions` 覆盖。
    int max_expansions{200000};
};

/// A* 全局规划器：8 邻接、禁止斜穿犄角、视线捷径化。
///
/// 依据设计 §7：2027 赛季 3V3 战场 12 m × 8 m，0.05 m/格只有 240 × 160 格，
/// A* 足够；θ\* 留到实测显示"折线导致跟随抖动"时再换。
class AstarPlanner : public GlobalPlanner {
  public:
    AstarPlanner() = default;
    explicit AstarPlanner(const AstarParams& params) : params_(params) {
    }

    NavError plan(const CostmapView& map, const Pose2& start, const Pose2& goal,
                  const PlanningOptions& options, Path* out) override;

    /// 当前参数（测试与调参用）。
    const AstarParams& params() const {
        return params_;
    }

  private:
    AstarParams params_{};

    // 复用的工作缓冲：每次 plan 只 resize / 重新填充，不再重新申请。
    // 原来每次规划要租约 500 KB（g_score + parent + closed），20 Hz 重规划会一直抖分配器
    // （审计 AS-7）。
    std::vector<double> g_score_{};
    std::vector<int> parent_{};
    std::vector<char> closed_{};
};

} // namespace nav
} // namespace foray

#endif // FORAY__NAV__ASTAR_PLANNER_HPP_
