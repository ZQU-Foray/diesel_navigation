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

#ifndef DIESEL__NAV__CONTROLLER_HPP_
#define DIESEL__NAV__CONTROLLER_HPP_

#include "diesel/nav/types.hpp"

namespace foray {
namespace nav {

/// 局部控制器接口：给当前位姿、一条路径和周期，出一个底盘速度指令。
///
/// **到达判定不在这里。** 分工写死如下（避免两边各写一套）：
/// - **控制器**：只负责"顺着路径走"，并在距终点小于 `stop_tolerance` 时把平移速度
///   按变化率收到零（**不是**一帧跳零）；若实现支持朝向对齐（见
///   `PursuitParams::align_heading_at_goal`），这一段输出的是角速度 `omega`，平移仍收零。
/// - **状态机**：拿 `NavGoal::xy_tolerance`（以及 `require_heading`）判 `kReached`，
///   再用 `NavStatus` 上报。**控制器永远不返回 `kReached`。**
///
/// 契约：
/// - **有状态**：可以记住上次指令做限幅/平滑；`reset()` 之后必须回到初始状态；
/// - 输出前必须做**幅值限幅**与**变化率限幅**，不许把超限值交给底盘；
/// - 不抛异常；内部失败返回具体原因，由状态机决定是否进入恢复。
class LocalController {
  public:
    virtual ~LocalController() = default;

    /// 计算一个控制周期的指令。
    /// @param pose     当前位姿（世界坐标）
    /// @param path     待跟随的路径（含起点）
    /// @param dt       距上一周期的时长（秒），必须为正
    /// @param out      输出指令
    /// @return 成功为 `NavError::kNone`；非法输入为 `kInvalidInput`
    virtual NavError compute(const Pose2& pose, const Path& path, double dt,
                             VelocityCommand* out) = 0;

    /// 清空内部状态（切换目标、进入恢复后必须调用）。
    virtual void reset() = 0;

    /// 停止容差（米）：距终点小于它时输出零速度。
    ///
    /// 注意：这**不是**到达判定——到达由状态机按 `NavGoal::xy_tolerance` 判。
    virtual double stop_tolerance() const = 0;
};

} // namespace nav
} // namespace foray

#endif // DIESEL__NAV__CONTROLLER_HPP_
