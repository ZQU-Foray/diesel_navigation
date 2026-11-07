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

NavEngine::NavEngine(const AstarParams& planner_params, const PursuitParams& controller_params)
    : planner_(planner_params), controller_(controller_params) {
}

void NavEngine::reset() {
    planner_ = AstarPlanner{};
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

    PlanningOptions options;
    path_.clear();
    const NavError error = planner_.plan(map, pose, goal.pose, options, &path_);
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

NavError NavEngine::replan(const CostmapView& map, const Pose2& pose) {
    if (replans_ >= options_.max_replans) {
        status_ = NavStatus::kFailed;
        last_error_ = NavError::kRecoveryExhausted;
        return last_error_;
    }
    ++replans_;
    status_ = NavStatus::kRecovering;

    Path fresh;
    PlanningOptions options;
    const NavError error = planner_.plan(map, pose, goal_.pose, options, &fresh);
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
    return heading_error <= 0.05;
}

NavError NavEngine::step(const CostmapView& map, const Pose2& pose, double dt,
                         VelocityCommand* out) {
    if (out == nullptr) {
        return NavError::kInvalidInput;
    }
    *out = VelocityCommand{};

    if (status_ == NavStatus::kReached || status_ == NavStatus::kFailed) {
        return last_error_;
    }
    if (status_ == NavStatus::kIdle) {
        last_error_ = NavError::kInvalidInput;
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

    // 4) 交给控制器。
    VelocityCommand command{};
    const NavError error = controller_.compute(pose, path_, dt, &command);
    last_error_ = error;
    if (error != NavError::kNone) {
        const NavError recovered = replan(map, pose);
        *out = VelocityCommand{};
        return recovered;
    }
    *out = command;
    return NavError::kNone;
}

} // namespace nav
} // namespace foray
