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

#include "foray/nav/nav_engine.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace foray {
namespace nav {

namespace {

double distance(const Vec2& a, const Vec2& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

double normalize_angle(double a) {
    constexpr double kPi = 3.14159265358979323846;
    while (a > kPi) {
        a -= 2.0 * kPi;
    }
    while (a <= -kPi) {
        a += 2.0 * kPi;
    }
    return a;
}

} // namespace

void NavEngine::build_safe_map(const CostmapView& map) {
    std::string error;
    // 每次都从原图重建：动态障碍会变，缓存一份旧的比不做更危险。
    if (safe_.width() != map.width() || safe_.height() != map.height() ||
        safe_.resolution() != map.resolution()) {
        if (!safe_.init(map.width(), map.height(), map.resolution(), map.origin(), &error)) {
            return;
        }
    }
    // 清成未知，再用静态层把非未知格刷进来（set_static_layer 内部会 reset）。
    safe_.set_static_layer(map, &error);
    const double radius = std::max(0.0, options_.robot_radius);
    if (radius > 0.0) {
        // 内切半径 = 车体半径：这个距离内一律致死，规划自然走中间。
        safe_.inflate(radius, radius * 1.2, 3.0);
    }
}

void NavEngine::build_guard_map(const CostmapView& map, double radius) {
    std::string error;
    if (guard_.width() != map.width() || guard_.height() != map.height() ||
        guard_.resolution() != map.resolution()) {
        guard_.init(map.width(), map.height(), map.resolution(), map.origin(), &error);
    }
    guard_.set_static_layer(map, &error);
    if (radius > 0.0) {
        guard_.inflate(radius, radius * 1.2, 3.0);
    }
}

void NavEngine::clear_disc(Costmap& map, const CostmapView& source, const Vec2& center,
                           double radius) {
    int cx = 0;
    int cy = 0;
    if (!map.world_to_grid(center, &cx, &cy)) {
        return;
    }
    const int span = static_cast<int>(std::ceil(radius / map.resolution()));
    for (int dy = -span; dy <= span; ++dy) {
        for (int dx = -span; dx <= span; ++dx) {
            // 原图上就是墙/未知，一律不动：清圈只负责"撤销膨胀"，不负责拆墙。
            const uint8_t original = source.cost_at(cx + dx, cy + dy);
            if (original == kCostLethal || original == kCostUnknown) {
                continue;
            }
            map.force_cost(cx + dx, cy + dy, kCostFree);
        }
    }
}

Vec2 NavEngine::escape_direction(const CostmapView& map, const Vec2& point) const {
    int cx = 0;
    int cy = 0;
    if (!map.world_to_grid(point, &cx, &cy)) {
        return Vec2{0.0, 0.0};
    }
    const int span = static_cast<int>(std::ceil(options_.robot_radius * 2.0 / map.resolution()));
    double best = std::numeric_limits<double>::max();
    Vec2 away{0.0, 0.0};
    for (int dy = -span; dy <= span; ++dy) {
        for (int dx = -span; dx <= span; ++dx) {
            if (map.cost_at(cx + dx, cy + dy) != kCostLethal) {
                continue;
            }
            Vec2 lethal{};
            if (!map.grid_to_world(cx + dx, cy + dy, &lethal)) {
                continue;
            }
            const double d = std::hypot(point.x - lethal.x, point.y - lethal.y);
            if (d < best) {
                best = d;
                const double inv = d > 1e-6 ? 1.0 / d : 0.0;
                away = Vec2{(point.x - lethal.x) * inv, (point.y - lethal.y) * inv};
            }
        }
    }
    return away;
}

double NavEngine::local_clearance(const CostmapView& map, const Vec2& point) const {
    int cx = 0;
    int cy = 0;
    if (!map.world_to_grid(point, &cx, &cy)) {
        return 0.0;
    }
    const int span = static_cast<int>(std::ceil(options_.robot_radius * 2.0 / map.resolution()));
    double best = std::numeric_limits<double>::max();
    for (int dy = -span; dy <= span; ++dy) {
        for (int dx = -span; dx <= span; ++dx) {
            if (map.cost_at(cx + dx, cy + dy) != kCostLethal) {
                continue;
            }
            Vec2 lethal{};
            if (!map.grid_to_world(cx + dx, cy + dy, &lethal)) {
                continue;
            }
            best = std::min(best, std::hypot(point.x - lethal.x, point.y - lethal.y));
        }
    }
    return best == std::numeric_limits<double>::max() ? options_.robot_radius * 2.0 : best;
}

bool NavEngine::segment_clear(const CostmapView& map, const Vec2& a, const Vec2& b) const {
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    const double step = std::max(0.01, map.resolution() * 0.5);
    const int samples = std::max(1, static_cast<int>(std::ceil(length / step)));
    for (int i = 0; i <= samples; ++i) {
        const double t = static_cast<double>(i) / samples;
        const double x = a.x + (b.x - a.x) * t;
        const double y = a.y + (b.y - a.y) * t;
        int ix = 0;
        int iy = 0;
        if (!map.world_to_grid(Vec2{x, y}, &ix, &iy)) {
            return false;
        }
        const uint8_t cost = map.cost_at(ix, iy);
        if (cost == kCostLethal || cost == kCostUnknown || cost == kCostReserved) {
            return false;
        }
    }
    return true;
}

NavEngine::NavEngine(const AstarParams& planner_params, const PursuitParams& controller_params)
    : planner_(planner_params), controller_(controller_params), planner_params_(planner_params),
      controller_params_(controller_params) {
}

void NavEngine::reset() {
    // 用构造时给的那份参数重建，不能退回编译期默认值（否则 reset 会悄悄改行为）。
    planner_ = AstarPlanner{planner_params_};
    controller_ = PursuitController{controller_params_};
    controller_.reset();
    goal_ = NavGoal{};
    path_.clear();
    status_ = NavStatus::kIdle;
    last_error_ = NavError::kNone;
    replans_ = 0;
    best_goal_distance_ = 0.0;
    stall_timer_s_ = 0.0;
    has_baseline_ = false;
}

NavError NavEngine::set_goal(const CostmapView& map, const NavGoal& goal, const Pose2& pose) {
    goal_ = goal;
    replans_ = 0;
    stall_timer_s_ = 0.0;
    has_baseline_ = false;
    status_ = NavStatus::kPlanning;

    // 控制器的停车容差必须比目标容差小：控制器在 stop_tolerance 处就收油，
    // 若它 ≥ goal.xy_tolerance，车会停在"几乎到"的位置，引擎永远判不到达，
    // 于是停滞→重规划→耗尽（2026-11-07 用 0.08 m 目标容差实测复现）。
    PursuitParams pursuit = controller_params_;
    if (pursuit.stop_tolerance >= goal.xy_tolerance) {
        pursuit.stop_tolerance = goal.xy_tolerance * 0.8;
        controller_ = PursuitController{pursuit};
    }

    path_.clear();
    const NavError error = plan_once(map, pose, goal.pose, &path_);
    last_error_ = error;
    if (error != NavError::kNone) {
        path_.clear();
        status_ = NavStatus::kFailed;
        return error;
    }
    // 让控制器从零速重新起步：换目标时保留上一段的速度没有意义。
    controller_.reset();
    status_ = NavStatus::kFollowing;
    best_goal_distance_ = distance(Vec2{pose.x, pose.y}, Vec2{goal.pose.x, goal.pose.y});
    has_baseline_ = true;
    return NavError::kNone;
}

NavError NavEngine::plan_once(const CostmapView& map, const Pose2& pose, const Pose2& goal,
                              Path* out) {
    // 目标本身落在墙里/未知区里，是**非法目标**，必须拒；只有在原图上本来就自由、
    // 只是被自己的膨胀层盖住的，才清一圈放行。
    int gx = 0;
    int gy = 0;
    if (!map.world_to_grid(Vec2{goal.x, goal.y}, &gx, &gy)) {
        return NavError::kStartOutsideMap;
    }
    const uint8_t goal_cost = map.cost_at(gx, gy);
    if (goal_cost == kCostLethal || goal_cost == kCostUnknown) {
        return NavError::kGoalInLethal;
    }

    build_safe_map(map);
    clear_disc(safe_, map, Vec2{goal.x, goal.y}, options_.robot_radius * 1.2);
    PlanningOptions options;
    NavError error = planner_.plan(safe_, pose, goal, options, out);
    if (error == NavError::kNone) {
        return error;
    }
    // 兜底：车贴着墙时，它自己那一格也被算成障碍，规划器于是"原地无法出发"。
    // 清掉车周围一圈再试一次——车真的在那儿，它必须能开出来。
    build_safe_map(map);
    clear_disc(safe_, map, Vec2{pose.x, pose.y}, options_.robot_radius * 1.3);
    clear_disc(safe_, map, Vec2{goal.x, goal.y}, options_.robot_radius * 1.2);
    out->clear();
    error = planner_.plan(safe_, pose, goal, options, out);
    return error;
}

NavError NavEngine::replan(const CostmapView& map, const Pose2& pose) {
    if (replans_ >= options_.max_replans) {
        status_ = NavStatus::kFailed;
        last_error_ = NavError::kRecoveryExhausted;
        return last_error_;
    }
    ++replans_;
    status_ = NavStatus::kRecovering;

    Path fresh;
    const NavError error = plan_once(map, pose, goal_.pose, &fresh);
    last_error_ = error;
    if (error != NavError::kNone) {
        status_ = NavStatus::kFailed;
        return error;
    }
    path_ = fresh;
    // 换路径时控制器要重新起步：旧指令是针对旧路径的。
    controller_.reset();
    status_ = NavStatus::kFollowing;
    stall_timer_s_ = 0.0;
    best_goal_distance_ = distance(Vec2{pose.x, pose.y}, Vec2{goal_.pose.x, goal_.pose.y});
    has_baseline_ = true;
    return NavError::kNone;
}

double NavEngine::distance_to_path(const Pose2& pose) const {
    if (path_.size() < 2) {
        return 0.0;
    }
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i + 1 < path_.size(); ++i) {
        const double ax = path_[i].x;
        const double ay = path_[i].y;
        const double bx = path_[i + 1].x;
        const double by = path_[i + 1].y;
        const double dx = bx - ax;
        const double dy = by - ay;
        const double len2 = dx * dx + dy * dy;
        double t = 0.0;
        if (len2 > 1e-18) {
            t = ((pose.x - ax) * dx + (pose.y - ay) * dy) / len2;
            t = std::max(0.0, std::min(1.0, t));
        }
        const double qx = ax + dx * t;
        const double qy = ay + dy * t;
        best = std::min(best, std::hypot(pose.x - qx, pose.y - qy));
    }
    return best;
}

bool NavEngine::reached(const Pose2& pose) const {
    const double d = distance(Vec2{pose.x, pose.y}, Vec2{goal_.pose.x, goal_.pose.y});
    if (d > goal_.xy_tolerance) {
        return false;
    }
    if (!goal_.require_heading) {
        return true;
    }
    const double heading_error = std::fabs(normalize_angle(goal_.pose.yaw - pose.yaw));
    return heading_error <= options_.heading_tolerance;
}

NavError NavEngine::step(const CostmapView& map, const Pose2& pose, double dt,
                         VelocityCommand* out) {
    if (out == nullptr) {
        return NavError::kInvalidInput;
    }
    *out = VelocityCommand{};

    // 非法输入必须挡住：dt<=0 会让控制器算不出方向，NaN 位姿会让所有比较都为假——
    // 不挡的话，NaN 位姿会被 reached() 判成"到了"（2026-11-07 实测：status 变成 kReached）。
    if (!(dt > 0.0) || !std::isfinite(dt) || !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
        !std::isfinite(pose.yaw)) {
        return NavError::kInvalidInput;
    }

    if (status_ == NavStatus::kReached || status_ == NavStatus::kFailed) {
        return last_error_;
    }
    if (status_ == NavStatus::kIdle) {
        last_error_ = NavError::kInvalidInput;
        *out = VelocityCommand{};
        return last_error_;
    }

    // 0) 路径没了（上一个周期重规划失败但没有被检查到）就先判失败，别拿空路径去算。
    if (path_.size() < 2) {
        status_ = NavStatus::kFailed;
        last_error_ = NavError::kNoPath;
        *out = VelocityCommand{};
        return last_error_;
    }

    // 1) 到点判定先做：到了就不该再踩油门。
    if (reached(pose)) {
        status_ = NavStatus::kReached;
        last_error_ = NavError::kNone;
        controller_.reset();
        *out = VelocityCommand{};
        return NavError::kNone;
    }

    // 2) 偏离路径太远 → 重规划（贴着旧路径硬走只会越走越偏）。
    if (path_.size() >= 2 && distance_to_path(pose) > options_.deviation_replan_dist) {
        const NavError error = replan(map, pose);
        if (error != NavError::kNone) {
            return error;
        }
    }

    // 3) 停滞判定：窗口内离目标没有推进，就当成卡住。
    const double goal_distance = distance(Vec2{pose.x, pose.y}, Vec2{goal_.pose.x, goal_.pose.y});
    if (!has_baseline_ || goal_distance < best_goal_distance_ - options_.stall_progress_m) {
        best_goal_distance_ = goal_distance;
        stall_timer_s_ = 0.0;
        has_baseline_ = true;
    } else {
        stall_timer_s_ += dt;
        if (stall_timer_s_ >= options_.stall_window_s) {
            last_error_ = NavError::kProgressStalled;
            const NavError error = replan(map, pose);
            if (error != NavError::kNone) {
                return error;
            }
        }
    }

    // 4) 交给控制器，但前视要"遇阻缩短"：对着远处的点硬走会切角撞墙。
    //    依次试 1.0 / 0.65 / 0.4 / 0.2 倍前视，取第一个前方安全的结果。
    // 护栏半径自适应：车已经贴得比护栏还近时，护栏按实际净空退一步——
    // 否则"我脚下就是禁区"，任何动作都判死，车原地罚站（2026-11-07 实测起点 0.12 m）。
    const double here_clear = local_clearance(map, Vec2{pose.x, pose.y});
    const double guard_radius = std::min(options_.robot_radius * 0.6,
                                         std::max(options_.robot_radius * 0.4, here_clear * 0.8));
    build_guard_map(map, guard_radius);
    const double scales[4] = {1.0, 0.65, 0.4, 0.2};
    VelocityCommand chosen{};
    bool safe_found = false;
    for (double scale : scales) {
        controller_.set_lookahead_scale(scale);
        VelocityCommand candidate{};
        const NavError err = controller_.compute(pose, path_, dt, &candidate);
        if (err != NavError::kNone) {
            continue;
        }
        const Vec2 here{pose.x, pose.y};
        const double v = std::hypot(candidate.vx, candidate.vy);
        const double horizon = std::max(0.10, v * options_.preview_time_s);
        const double scale_dir = v > 1e-9 ? horizon / v : 0.0;
        const Vec2 ahead{pose.x + candidate.vx * scale_dir, pose.y + candidate.vy * scale_dir};
        if (segment_clear(guard_, here, ahead)) {
            chosen = candidate;
            safe_found = true;
            break;
        }
    }
    controller_.set_lookahead_scale(1.0);

    if (!safe_found) {
        // 先退再跟：车已经贴墙、路径又贴着墙走时，直接跟只会撞——先朝"远离最近障碍"的方向挪一点。
        {
            const Vec2 here{pose.x, pose.y};
            const Vec2 away = escape_direction(map, here);
            if (away.x != 0.0 || away.y != 0.0) {
                const double speed = controller_.params().max_linear * 0.3;
                VelocityCommand back{away.x * speed, away.y * speed, 0.0};
                const Vec2 ahead{here.x + back.vx * dt * 4.0, here.y + back.vy * dt * 4.0};
                if (segment_clear(map, here, ahead) &&
                    local_clearance(map, ahead) > local_clearance(map, here)) {
                    *out = back;
                    last_error_ = NavError::kNone;
                    return NavError::kNone;
                }
            }
        }

        // 到点放宽：终点本身可能就贴着墙，最后这一段必须允许走进去，
        // 否则会停在离终点十几厘米处永远判不到达（2026-11-07 trace 实测：停在 16.6 cm）。
        const double to_goal = std::hypot(pose.x - goal_.pose.x, pose.y - goal_.pose.y);
        if (to_goal <= std::max(0.35, options_.robot_radius * 1.5)) {
            controller_.set_lookahead_scale(1.0);
            VelocityCommand approach{};
            if (controller_.compute(pose, path_, dt, &approach) == NavError::kNone) {
                *out = approach;
                last_error_ = NavError::kNone;
                return NavError::kNone;
            }
        }
        // 缩到最小前视也撞：这一帧停住，重规划交给停滞判定（别在这里把次数烧光）。
        controller_.reset();
        last_error_ = NavError::kProgressStalled;
        *out = VelocityCommand{};
        return NavError::kNone;
    }

    last_error_ = NavError::kNone;
    *out = chosen;
    return NavError::kNone;
}

} // namespace nav
} // namespace foray
