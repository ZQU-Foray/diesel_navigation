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

#include "foray/nav/pursuit_controller.hpp"

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
    if (!condition) {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

foray::nav::Path make_straight_path() {
    foray::nav::Path path;
    for (int i = 0; i <= 40; ++i) {
        path.push_back(foray::nav::Pose2{0.05 * i, 0.0, 0.0});
    }
    return path;
}

/// 用一个假的积分器推演：位姿按指令前进 dt，看能不能贴到路径上。
void test_tracking_converges() {
    foray::nav::PursuitController controller;
    const foray::nav::Path path = make_straight_path();
    // 起点偏离路径 0.5 m
    foray::nav::Pose2 pose{0.2, 0.5, 0.0};
    const double dt = 0.05;
    for (int step = 0; step < 400; ++step) {
        foray::nav::VelocityCommand command;
        const auto error = controller.compute(pose, path, dt, &command);
        check(error == foray::nav::NavError::kNone, "tracking: compute ok");
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
    foray::nav::PursuitParams params;
    params.max_linear = 1.0;
    params.max_accel = 2.0;
    foray::nav::PursuitController controller(params);
    const foray::nav::Path path = make_straight_path();
    foray::nav::Pose2 pose{0.1, 0.1, 0.0};
    const double dt = 0.1;
    foray::nav::VelocityCommand previous{};
    for (int step = 0; step < 50; ++step) {
        foray::nav::VelocityCommand command;
        check(controller.compute(pose, path, dt, &command) == foray::nav::NavError::kNone,
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
    foray::nav::PursuitController controller;
    const foray::nav::Path path = make_straight_path();
    const foray::nav::Vec2 goal{path.back().x, path.back().y};
    const foray::nav::Pose2 pose{goal.x + 0.05, goal.y, 0.0}; // 在容差内
    foray::nav::VelocityCommand command{1.0, 1.0, 0.0};
    check(controller.compute(pose, path, 0.05, &command) == foray::nav::NavError::kNone,
          "goal: ok");
    check(std::fabs(command.vx) < 1e-12 && std::fabs(command.vy) < 1e-12, "goal: zero velocity");
}

void test_invalid_inputs() {
    foray::nav::PursuitController controller;
    foray::nav::VelocityCommand command;
    foray::nav::Path empty;
    check(controller.compute(foray::nav::Pose2{}, empty, 0.05, &command) ==
              foray::nav::NavError::kInvalidInput,
          "invalid: empty path");
    const foray::nav::Path path = make_straight_path();
    check(controller.compute(foray::nav::Pose2{}, path, 0.0, &command) ==
              foray::nav::NavError::kInvalidInput,
          "invalid: dt must be positive");
    check(controller.compute(foray::nav::Pose2{}, path, 0.05, nullptr) ==
              foray::nav::NavError::kInvalidInput,
          "invalid: null out");
}

void test_reset_clears_state() {
    foray::nav::PursuitController controller;
    const foray::nav::Path path = make_straight_path();
    foray::nav::VelocityCommand command;
    controller.compute(foray::nav::Pose2{0.1, 0.0, 0.0}, path, 0.05, &command);
    controller.reset();
    // reset 之后第一帧不受变化率限幅约束：可以立刻给出较大的期望速度
    foray::nav::PursuitController fresh;
    foray::nav::VelocityCommand a;
    foray::nav::VelocityCommand b;
    controller.compute(foray::nav::Pose2{0.1, 0.0, 0.0}, path, 0.05, &a);
    fresh.compute(foray::nav::Pose2{0.1, 0.0, 0.0}, path, 0.05, &b);
    check(std::fabs(a.vx - b.vx) < 1e-9, "reset: behaves like a fresh controller");
}

} // namespace

int main() {
    test_tracking_converges();
    test_limits();
    test_goal_zero();
    test_invalid_inputs();
    test_reset_clears_state();

    if (g_failures == 0) {
        std::printf("ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
}
