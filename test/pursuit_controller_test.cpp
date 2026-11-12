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

// 纯追踪控制器测试：直线跟踪收敛、幅值限幅、变化率限幅、到点归零、非法输入。

#include <cmath>
#include <cstdio>

#include "diesel/nav/pursuit_controller.hpp"

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
    if (!condition) {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

diesel::nav::Path make_straight_path() {
    diesel::nav::Path path;
    for (int i = 0; i <= 40; ++i) {
        path.push_back(diesel::nav::Pose2{0.05 * i, 0.0, 0.0});
    }
    return path;
}

/// 用一个假的积分器推演：位姿按指令前进 dt，看能不能贴到路径上。
void test_tracking_converges() {
    diesel::nav::PursuitController controller;
    const diesel::nav::Path path = make_straight_path();
    // 起点偏离路径 0.5 m
    diesel::nav::Pose2 pose{0.2, 0.5, 0.0};
    const double dt = 0.05;
    for (int step = 0; step < 400; ++step) {
        diesel::nav::VelocityCommand command;
        const auto error = controller.compute(pose, path, dt, &command);
        check(error == diesel::nav::NavError::kNone, "tracking: compute ok");
        pose.x += command.vx * dt;
        pose.y += command.vy * dt;
        if (std::fabs(pose.y) < 0.01 && pose.x > 1.0) {
            break;
        }
    }
    check(std::fabs(pose.y) < 0.05, "tracking: converges onto the path");
    check(pose.x > 1.0, "tracking: makes progress along the path");
}

void test_limits() {
    diesel::nav::PursuitParams params;
    params.max_linear = 1.0;
    params.max_accel = 2.0;
    diesel::nav::PursuitController controller(params);
    const diesel::nav::Path path = make_straight_path();
    diesel::nav::Pose2 pose{0.1, 0.1, 0.0};
    const double dt = 0.1;
    diesel::nav::VelocityCommand previous{};
    for (int step = 0; step < 50; ++step) {
        diesel::nav::VelocityCommand command;
        check(controller.compute(pose, path, dt, &command) == diesel::nav::NavError::kNone,
              "limits: ok");
        check(std::hypot(command.vx, command.vy) <= params.max_linear + 1e-9,
              "limits: magnitude capped");
        const double delta = std::hypot(command.vx - previous.vx, command.vy - previous.vy);
        check(delta <= params.max_accel * dt + 1e-9, "limits: rate capped");
        previous = command;
        pose.x += command.vx * dt;
        pose.y += command.vy * dt;
    }
}

void test_goal_zero() {
    diesel::nav::PursuitController controller;
    const diesel::nav::Path path = make_straight_path();
    const diesel::nav::Vec2 goal{path.back().x, path.back().y};
    const diesel::nav::Pose2 pose{goal.x + 0.05, goal.y, 0.0}; // 在容差内
    diesel::nav::VelocityCommand command{1.0, 1.0, 0.0};
    check(controller.compute(pose, path, 0.05, &command) == diesel::nav::NavError::kNone,
          "goal: ok");
    check(std::fabs(command.vx) < 1e-12 && std::fabs(command.vy) < 1e-12, "goal: zero velocity");
}

void test_invalid_inputs() {
    diesel::nav::PursuitController controller;
    diesel::nav::VelocityCommand command;
    diesel::nav::Path empty;
    check(controller.compute(diesel::nav::Pose2{}, empty, 0.05, &command) ==
              diesel::nav::NavError::kInvalidInput,
          "invalid: empty path");
    const diesel::nav::Path path = make_straight_path();
    check(controller.compute(diesel::nav::Pose2{}, path, 0.0, &command) ==
              diesel::nav::NavError::kInvalidInput,
          "invalid: dt must be positive");
    check(controller.compute(diesel::nav::Pose2{}, path, 0.05, nullptr) ==
              diesel::nav::NavError::kInvalidInput,
          "invalid: null out");
}

void test_reset_clears_state() {
    diesel::nav::PursuitController controller;
    const diesel::nav::Path path = make_straight_path();
    diesel::nav::VelocityCommand command;
    controller.compute(diesel::nav::Pose2{0.1, 0.0, 0.0}, path, 0.05, &command);
    controller.reset();
    // reset 之后第一帧不受变化率限幅约束：可以立刻给出较大的期望速度
    diesel::nav::PursuitController fresh;
    diesel::nav::VelocityCommand a;
    diesel::nav::VelocityCommand b;
    controller.compute(diesel::nav::Pose2{0.1, 0.0, 0.0}, path, 0.05, &a);
    fresh.compute(diesel::nav::Pose2{0.1, 0.0, 0.0}, path, 0.05, &b);
    check(std::fabs(a.vx - b.vx) < 1e-9, "reset: behaves like a fresh controller");
}

// ---- 以下是 2026-11-03 审计（PC-1 / PC-2 / PC-3）补的回归测试 ----
// 原来的用例都从 path[0] 出发、只走一两步，正好绕开了"车已前进"这一段。

/// PC-1：20 m 直线，车必须真的走到终点，且全程不许报错、不许倒车。
void test_reaches_goal_on_long_straight() {
    diesel::nav::Path path;
    for (int i = 0; i <= 200; ++i) {
        path.push_back(diesel::nav::Pose2{0.1 * i, 0.0, 0.0});
    }
    diesel::nav::PursuitController controller;
    diesel::nav::Pose2 pose{0.0, 0.0, 0.0};
    const double dt = 0.05;
    const diesel::nav::Vec2 goal{path.back().x, path.back().y};
    bool ever_negative = false;
    bool ever_error = false;
    for (int step = 0; step < 4000; ++step) {
        diesel::nav::VelocityCommand command;
        if (controller.compute(pose, path, dt, &command) != diesel::nav::NavError::kNone) {
            ever_error = true;
            break;
        }
        // 直线路径上沿 x 前进，vx 一旦变负就是"倒车"——PC-1 的典型症状
        if (command.vx < -1e-9) {
            ever_negative = true;
        }
        pose.x += command.vx * dt;
        pose.y += command.vy * dt;
        if (std::hypot(pose.x - goal.x, pose.y - goal.y) < 0.20) {
            break;
        }
    }
    check(!ever_error, "long straight: never fails");
    check(!ever_negative, "long straight: never drives backwards");
    check(std::hypot(pose.x - goal.x, pose.y - goal.y) < 0.20,
          "long straight: actually reaches the goal");
}

/// PC-2：直线上的曲率必须约等于 0，速度缩放不能被砍半。
void test_straight_line_has_no_curvature() {
    diesel::nav::PursuitController controller;
    const diesel::nav::Path path = make_straight_path();
    for (int step = 0; step < 200; ++step) {
        diesel::nav::Pose2 pose{0.05 * step, 0.0, 0.0};
        diesel::nav::VelocityCommand command;
        check(controller.compute(pose, path, 0.05, &command) == diesel::nav::NavError::kNone,
              "straight: ok");
        // 直线且速度未到顶时限速只能来自"到点减速"，不该在中段就掉到一半
        if (pose.x > 0.5 && pose.x < 1.5) {
            check(std::hypot(command.vx, command.vy) > 0.5 * controller.params().max_linear,
                  "straight: not halved by bogus curvature");
        }
    }
}

/// PC-3：到点停车也必须过变化率限幅，不许一帧从巡航速度跳到 0。
void test_goal_stop_respects_rate_limit() {
    diesel::nav::PursuitParams params;
    params.max_linear = 2.5;
    params.max_accel = 6.0;
    diesel::nav::PursuitController controller(params);
    const diesel::nav::Path path = make_straight_path();
    const double dt = 0.05;

    // 先跑起来，攒一点速度
    diesel::nav::VelocityCommand command;
    diesel::nav::Pose2 pose{0.0, 0.0, 0.0};
    for (int step = 0; step < 20; ++step) {
        controller.compute(pose, path, dt, &command);
        pose.x += command.vx * dt;
    }
    const double speed_before = std::hypot(command.vx, command.vy);
    check(speed_before > 0.5, "stop: got up to speed first");

    // 一步跨进到点容差内
    const diesel::nav::Vec2 goal{path.back().x, path.back().y};
    diesel::nav::Pose2 near_goal{goal.x - 0.01, goal.y, 0.0};
    diesel::nav::VelocityCommand stop{};
    check(controller.compute(near_goal, path, dt, &stop) == diesel::nav::NavError::kNone,
          "stop: ok");
    const double drop = speed_before - std::hypot(stop.vx, stop.vy);
    check(drop <= params.max_accel * dt + 1e-9, "stop: rate limited");
}

/// PC-4：打开 align_heading_at_goal 后，到点阶段要输出 omega（原来恒为 0）。
void test_heading_alignment_at_goal() {
    diesel::nav::PursuitParams params;
    params.align_heading_at_goal = true;
    diesel::nav::PursuitController controller(params);
    diesel::nav::Path path;
    path.push_back(diesel::nav::Pose2{0.0, 0.0, 0.0});
    path.push_back(diesel::nav::Pose2{1.0, 0.0, 1.0}); // 末端要求朝向 1 rad
    const diesel::nav::Pose2 pose{1.0, 0.0, 0.0};      // 位置到了，朝向差 1 rad
    diesel::nav::VelocityCommand command;
    check(controller.compute(pose, path, 0.05, &command) == diesel::nav::NavError::kNone,
          "align: ok");
    check(std::fabs(command.omega) > 1e-6, "align: omega is produced when heading off");
}

} // namespace

int main() {
    test_tracking_converges();
    test_limits();
    test_goal_zero();
    test_invalid_inputs();
    test_reset_clears_state();
    test_reaches_goal_on_long_straight();
    test_straight_line_has_no_curvature();
    test_goal_stop_respects_rate_limit();
    test_heading_alignment_at_goal();

    if (g_failures == 0) {
        std::printf("ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
}
