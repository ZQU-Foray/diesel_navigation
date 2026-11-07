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

#ifndef FORAY__NAV__NAV_ENGINE_HPP_
#define FORAY__NAV__NAV_ENGINE_HPP_

#include "foray/nav/astar_planner.hpp"
#include "foray/nav/costmap_view.hpp"
#include "foray/nav/pursuit_controller.hpp"
#include "foray/nav/types.hpp"

namespace foray {
namespace nav {

/// 导航引擎：把「规划 → 跟踪 → 判到达 → 必要时重规划」串成一条主循环。
///
/// 这一层**不依赖 ROS**，也不碰话题、时钟、日志——外面喂位姿和 `dt`，它吐速度指令。
/// ROS 适配层（后续）只做话题转换，调 `set_goal()` 与 `step()` 即可。
///
/// 状态流转：`kIdle` → `kPlanning` → `kFollowing` → `kReached` / `kFailed`。
/// 中途偏离路径或停滞过久会重新规划，次数用尽则 `kFailed` + `kProgressStalled`。
class NavEngine {
  public:
    /// 重规划与停滞判定参数。
    struct Options {
        /// 车离路径超过这个距离才重规划（米）。太小会抖，太大会贴着旧路径死走。
        double deviation_replan_dist{0.50};
        /// 单次任务里最多重规划几次。
        int max_replans{3};
        /// 判停滞的时间窗（秒）：这么久里离目标没有推进，就算卡住。
        double stall_window_s{2.0};
        /// 判停滞的最小推进量（米）。
        double stall_progress_m{0.05};
    };

    NavEngine() = default;
    NavEngine(const AstarParams& planner_params, const PursuitParams& controller_params);

    /// 设目标并立刻规划一次。
    /// @return 规划成功返回 `kNone`；失败返回原因，此时状态为 `kFailed`。
    NavError set_goal(const CostmapView& map, const NavGoal& goal, const Pose2& pose);

    /// 推进一个控制周期。
    /// @param out 非空；写本周期速度指令。到达或失败时写零速度。
    NavError step(const CostmapView& map, const Pose2& pose, double dt, VelocityCommand* out);

    NavStatus status() const {
        return status_;
    }
    const Path& path() const {
        return path_;
    }
    NavGoal goal() const {
        return goal_;
    }
    int replan_count() const {
        return replans_;
    }
    NavError last_error() const {
        return last_error_;
    }
    Options& options() {
        return options_;
    }
    const Options& options() const {
        return options_;
    }
    /// 回到初始状态（路径清空、控制器归零、计数归零）。
    void reset();

  private:
    /// 重新规划；成功则更新 `path_` 并回到 `kFollowing`。
    NavError replan(const CostmapView& map, const Pose2& pose);

    /// 车到路径的最短距离（用于偏离判定）。
    double distance_to_path(const Pose2& pose) const;

    /// 是否已满足到达条件。
    bool reached(const Pose2& pose) const;

    AstarPlanner planner_{};
    PursuitController controller_{};
    Options options_{};
    NavGoal goal_{};
    Path path_{};
    NavStatus status_{NavStatus::kIdle};
    NavError last_error_{NavError::kNone};
    int replans_{0};
    /// 停滞判定的基准：上次记录推进时的目标距离与时间累计。
    double best_goal_distance_{0.0};
    double stall_timer_s_{0.0};
    bool has_baseline_{false};
};

} // namespace nav
} // namespace foray

#endif // FORAY__NAV__NAV_ENGINE_HPP_
