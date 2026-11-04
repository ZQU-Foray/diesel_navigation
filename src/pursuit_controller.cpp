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

#include "foray/nav/pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace foray {
namespace nav {
namespace {

double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

double distance(const Vec2& a, const Vec2& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

/// 把角度折回 (-pi, pi]。
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

/// 把 `p` 投到线段 [a,b] 上的参数 t（夹在 0..1）。
double project_param(const Vec2& a, const Vec2& b, const Vec2& p) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    if (len2 < 1e-18) {
        return 0.0;
    }
    return clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0);
}

/// 从 `from` 沿路径向前走 `lookahead` 米，返回那个点（路径不够长就给终点）。
/// @note 必须先找"车在路径上的位置"再往前走；从 path[0] 起算会让车过了目标点就倒车
///       （2026-11-03 审计 PC-1，实测只走到 1.365 m）。
Vec2 lookahead_point(const Path& path, const Vec2& from, double lookahead) {
    // 1) 各段累计弧长
    const size_t n = path.size();
    std::vector<double> cum(n, 0.0);
    for (size_t i = 0; i + 1 < n; ++i) {
        cum[i + 1] = cum[i] + distance({path[i].x, path[i].y}, {path[i + 1].x, path[i + 1].y});
    }

    // 2) 找车投影到路径上的弧长 s0
    double s0 = 0.0;
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i + 1 < n; ++i) {
        const Vec2 a{path[i].x, path[i].y};
        const Vec2 b{path[i + 1].x, path[i + 1].y};
        const double t = project_param(a, b, from);
        const Vec2 q{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
        const double d2 = (q.x - from.x) * (q.x - from.x) + (q.y - from.y) * (q.y - from.y);
        if (d2 < best) {
            best = d2;
            s0 = cum[i] + t * distance(a, b);
        }
    }

    // 3) 沿弧长 s0 + lookahead 取点
    const double target_s = s0 + lookahead;
    for (size_t i = 0; i + 1 < n; ++i) {
        if (cum[i + 1] >= target_s) {
            const double seg = cum[i + 1] - cum[i];
            const double t = seg > 1e-12 ? (target_s - cum[i]) / seg : 0.0;
            const Vec2 a{path[i].x, path[i].y};
            const Vec2 b{path[i + 1].x, path[i + 1].y};
            return Vec2{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
        }
    }
    return Vec2{path.back().x, path.back().y};
}

/// 用"前视点方向"和"更远一点的方向"夹出的夹角估计曲率（1/m）。
double estimate_curvature(const Path& path, const Vec2& from, double forward_dist) {
    const Vec2 near_point = lookahead_point(path, from, forward_dist);
    const Vec2 far_point = lookahead_point(path, from, forward_dist * 2.0);
    const double dx1 = near_point.x - from.x;
    const double dy1 = near_point.y - from.y;
    const double dx2 = far_point.x - near_point.x;
    const double dy2 = far_point.y - near_point.y;
    const double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    const double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    if (len1 < 1e-6 || len2 < 1e-6) {
        return 0.0;
    }
    const double cross = dx1 * dy2 - dy1 * dx2;
    const double dot = dx1 * dx2 + dy1 * dy2;
    const double angle = std::fabs(std::atan2(cross, dot));
    return angle / std::max(1e-6, (len1 + len2) * 0.5);
}

} // namespace

void PursuitController::reset() {
    last_command_ = VelocityCommand{};
    last_scale_ = 1.0;
    has_last_ = false;
}

NavError PursuitController::compute(const Pose2& pose, const Path& path, double dt,
                                    VelocityCommand* out) {
    if (out == nullptr) {
        return NavError::kInvalidInput;
    }
    if (path.size() < 2 || !(dt > 0.0) || !std::isfinite(pose.x) || !std::isfinite(pose.y)) {
        return NavError::kInvalidInput;
    }

    const Vec2 current{pose.x, pose.y};
    const Vec2 goal{path.back().x, path.back().y};
    const double goal_distance = distance(current, goal);

    // 到点：目标速度为零，但仍要过变化率限幅——不许一帧从巡航速度直接跌到 0。
    // （2026-11-03 审计 PC-3：实测 2.500 m/s 单帧阶跃，契约上限 max_accel*dt = 0.300。）
    if (goal_distance <= params_.stop_tolerance) {
        // 到位后要对朝向：只转不走（审计 PC-4——`omega` 原来恒为 0）。
        if (params_.align_heading_at_goal) {
            const double heading_error = normalize_angle(path.back().yaw - pose.yaw);
            if (std::fabs(heading_error) > params_.heading_tolerance) {
                const double max_delta = params_.max_accel * dt;
                const double linear = std::hypot(last_command_.vx, last_command_.vy);
                VelocityCommand turn{};
                if (linear > max_delta && linear > 1e-12) {
                    const double k = max_delta / linear;
                    turn.vx = last_command_.vx * (1.0 - k);
                    turn.vy = last_command_.vy * (1.0 - k);
                }
                turn.omega = clamp(heading_error * params_.heading_gain, -params_.max_omega,
                                   params_.max_omega);
                last_command_ = turn;
                last_scale_ = 0.0;
                has_last_ = true;
                *out = turn;
                return NavError::kNone;
            }
        }
        const double max_delta = params_.max_accel * dt;
        const double delta = std::hypot(last_command_.vx, last_command_.vy);
        VelocityCommand stop{};
        if (delta > max_delta && delta > 1e-12) {
            const double k = max_delta / delta;
            stop.vx = last_command_.vx * (1.0 - k);
            stop.vy = last_command_.vy * (1.0 - k);
        }
        last_command_ = stop;
        last_scale_ = 0.0;
        has_last_ = true;
        *out = stop;
        return NavError::kNone;
    }

    // 1) 前视距离：速度自适应（用上次的速度估），再夹到 [min, max]。
    const double speed_now = std::hypot(last_command_.vx, last_command_.vy);
    double lookahead =
        clamp(speed_now * params_.lookahead_time, params_.lookahead_min, params_.lookahead_max);
    if (!has_last_) {
        lookahead = params_.lookahead_min;
    }
    const Vec2 target = lookahead_point(path, current, lookahead);

    // 2) 期望方向
    const double dx = target.x - current.x;
    const double dy = target.y - current.y;
    const double norm = std::sqrt(dx * dx + dy * dy);
    if (norm < 1e-9) {
        return NavError::kInvalidInput;
    }

    // 3) 速度缩放：曲率降速 + 到点减速，取更保守的那个
    double scale = 1.0;
    const double curvature = estimate_curvature(path, current, params_.curvature_forward_dist);
    if (curvature > params_.curvature_min) {
        const double span = std::max(1e-6, params_.curvature_max - params_.curvature_min);
        const double ratio = clamp((curvature - params_.curvature_min) / span, 0.0, 1.0);
        scale = std::min(scale, 1.0 - ratio * (1.0 - params_.reduction_ratio_at_high_curvature));
    }
    if (goal_distance < params_.approach_scaling_dist) {
        const double ratio =
            clamp(goal_distance / std::max(1e-6, params_.approach_scaling_dist), 0.0, 1.0);
        const double approach_speed =
            std::max(params_.min_approach_speed, params_.max_linear * ratio);
        scale = std::min(scale, approach_speed / std::max(1e-6, params_.max_linear));
    }
    last_scale_ = clamp(scale, 0.0, 1.0);

    // 4) 期望速度（全向：方向即前视点方向，不转车头）
    VelocityCommand desired{};
    desired.vx = dx / norm * params_.max_linear * last_scale_;
    desired.vy = dy / norm * params_.max_linear * last_scale_;

    // 5) 变化率限幅：按**矢量差**限，不是按轴——按轴限会让斜向的加速度放大到 sqrt(2) 倍。
    {
        // 第一帧也限：从零速开始，不能一上来就给满速（物理上叫冲出去）。
        const double max_delta = params_.max_accel * dt;
        const double dvx = desired.vx - last_command_.vx;
        const double dvy = desired.vy - last_command_.vy;
        const double delta = std::hypot(dvx, dvy);
        if (delta > max_delta && delta > 1e-12) {
            const double k = max_delta / delta;
            desired.vx = last_command_.vx + dvx * k;
            desired.vy = last_command_.vy + dvy * k;
        }
    }

    // 6) 幅值限幅（变化率限幅后仍要卡一次）
    const double magnitude = std::hypot(desired.vx, desired.vy);
    if (magnitude > params_.max_linear) {
        const double k = params_.max_linear / magnitude;
        desired.vx *= k;
        desired.vy *= k;
    }

    last_command_ = desired;
    has_last_ = true;
    *out = desired;
    return NavError::kNone;
}

} // namespace nav
} // namespace foray
