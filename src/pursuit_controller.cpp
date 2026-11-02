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

/// 从 `from` 沿路径向前走 `lookahead` 米，返回那个点（路径不够长就给终点）。
Vec2 lookahead_point(const Path& path, const Vec2& from, double lookahead) {
    Vec2 best{path.back().x, path.back().y};
    double walked = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const Vec2 a{path[i].x, path[i].y};
        const Vec2 b{path[i + 1].x, path[i + 1].y};
        const double segment = distance(a, b);
        if (segment < 1e-9) {
            continue;
        }
        if (walked + segment >= lookahead) {
            const double remain = lookahead - walked;
            const double t = remain / segment;
            return Vec2{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
        }
        walked += segment;
        best = b;
    }
    (void)from;
    return best;
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

    // 到点：输出零速度，交给状态机判到达。
    if (goal_distance <= params_.stop_tolerance) {
        last_command_ = VelocityCommand{};
        last_scale_ = 0.0;
        has_last_ = true;
        *out = last_command_;
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
