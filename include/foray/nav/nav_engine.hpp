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
#include "foray/nav/costmap.hpp"
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
        /// 车体半径（米，含余量）：规划与安全自检都按它留余量。实际车体约 0.25 m，
        /// 默认 0.35 是留出来的缓冲——纯追踪切角会吃掉约 0.1 m（2026-11-07 实测）。
        /// 纯追踪会切角，路径贴着墙走就会蹭上去——所以规划用"膨胀过的地图"，
        /// 让路径自己走中间（2026-11-07 实测：不加这条，切角处离墙只有 7 mm）。
        double robot_radius{0.25};
        /// 车离路径超过这个距离才重规划（米）。太小会抖，太大会贴着旧路径死走。
        double deviation_replan_dist{0.50};
        /// 单次任务里最多重规划几次。
        int max_replans{3};
        /// 判停滞的时间窗（秒）：这么久里离目标没有推进，就算卡住。
        double stall_window_s{2.0};
        /// 判停滞的最小推进量（米）。
        double stall_progress_m{0.05};
        /// 安全自检往前看多久（秒）：按当前指令推演这段路程，看会不会撞上去。
        double preview_time_s{0.15};
        /// 撞上去就按这个比例减速（爬行），还不行为零速并重规划。
        double creep_scale{0.25};
        /// 到点判定的朝向容差（弧度），只在 `NavGoal::require_heading` 为真时用。
        double heading_tolerance{0.05};
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
    /// 规划一次，带"起点在膨胀层里"的兜底：清掉车周围一圈再试一次。
    /// 车已经贴墙时，规划器会把自己的格子当障碍，不兜底就永远发不出去。
    NavError plan_once(const CostmapView& map, const Pose2& pose, const Pose2& goal, Path* out);

    /// 重新规划；成功则更新 `path_` 并回到 `kFollowing`。
    NavError replan(const CostmapView& map, const Pose2& pose);

    /// 把外部地图拷一份，并按车体半径膨胀成"安全层"：规划与自检都在它上面做。
    void build_safe_map(const CostmapView& map);

    /// 刹车用的"护栏地图"：只按半个车体半径膨胀——留一点余量，但不至于把窄口堵死。
    void build_guard_map(const CostmapView& map, double radius);
    /// 把 `center` 周围 `radius` 内**被膨胀层盖住**的格子清成自由（起点/终点兜底用）。
    /// @param source 原图：原图上就是致死/未知的格子**不清**——否则会在薄墙上打出洞，
    ///               规划出的路径直接穿墙（2026-11-07 实测 33 条路径里 13 条穿墙）。
    static void clear_disc(Costmap& map, const CostmapView& source, const Vec2& center,
                           double radius);

    /// 某点附近的最小净空（米）：扫半径内的小窗口，O((2r/res)^2)，够便宜。
    double local_clearance(const CostmapView& map, const Vec2& point) const;

    /// 远离最近障碍的方向（单位向量）；四周都没有障碍时返回 {0,0}。
    Vec2 escape_direction(const CostmapView& map, const Vec2& point) const;

    /// 从 `a` 到 `b` 这一段（按格子采样）是否没有致死/未知格。
    bool segment_clear(const CostmapView& map, const Vec2& a, const Vec2& b) const;

    /// 车到路径的最短距离（用于偏离判定）。
    double distance_to_path(const Pose2& pose) const;

    /// 是否已满足到达条件。
    bool reached(const Pose2& pose) const;

    AstarPlanner planner_{};
    PursuitController controller_{};
    /// 构造时给的参数：`reset()` 要还回这一份，而不是编译期默认值。
    AstarParams planner_params_{};
    PursuitParams controller_params_{};
    Options options_{};
    /// 按车体半径膨胀过的地图副本：规划与安全自检都用它，不用外部原图。
    Costmap safe_{};
    Costmap guard_{};
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
