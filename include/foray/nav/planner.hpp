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

#ifndef DIESEL__NAV__PLANNER_HPP_
#define DIESEL__NAV__PLANNER_HPP_

#include <cstdint>

#include "diesel/nav/costmap_view.hpp"
#include "diesel/nav/types.hpp"

namespace foray {
namespace nav {

/// 规划预算：把"能不能在要求时间内返回"写进契约，而不是留给实现自觉。
struct PlanningOptions {
    /// 最多展开多少个栅格节点；<= 0 表示用实现内定的上界。
    int max_expansions{120000};
    /// 墙钟截止时间（纳秒，`steady_clock` 口径）；0 表示不设截止。
    int64_t deadline_ns{0};
};

/// 全局规划器接口：给起点、终点、一张只读代价图和预算，出一条路径。
///
/// 契约：
/// - **纯函数式**：不持有跨调用的规划状态；同一输入必须给同一输出；
/// - **代价必须当边权重用**：`cost_at()` 的值要参与行走代价的计算，
///   不得二值化成"能走 / 不能走"——否则代价场（膨胀、偏好）全部失效；
/// - 失败必须给出具体原因（`kNoPath` / `kStartOutsideMap` / `kGoalInLethal` / `kInvalidInput`）；
/// - 目标落在致死格时**不搜索**，直接 `kGoalInLethal`；
/// - **必须尊重预算**：超出 `max_expansions` 或 `deadline_ns` 就停下并返回
///   `kNoPath`（本调用内可达性未证明），**不许阻塞**；
/// - 不修改 `CostmapView`；不抛异常。
class GlobalPlanner {
  public:
    virtual ~GlobalPlanner() = default;

    /// 规划一条从 `start` 到 `goal` 的路径。
    /// @param map     只读代价快照
    /// @param start   起点（世界坐标）
    /// @param goal    目标（世界坐标）
    /// @param out     输出路径；成功时至少含起点与终点，失败时不保证内容
    /// @param options 规划预算
    /// @return 成功为 `NavError::kNone`
    virtual NavError plan(const CostmapView& map, const Pose2& start, const Pose2& goal,
                          const PlanningOptions& options, Path* out) = 0;
};

} // namespace nav
} // namespace foray

#endif // DIESEL__NAV__PLANNER_HPP_
