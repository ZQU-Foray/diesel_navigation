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

// 导航引擎测试：能不能真的走到终点、失败要带原因、偏离要重规划、停滞要重规划。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "foray/nav/costmap.hpp"
#include "foray/nav/nav_engine.hpp"

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
    if (!condition) {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

constexpr int kW = 80;
constexpr int kH = 60;
constexpr double kRes = 0.05;
constexpr double kDt = 0.05;

foray::nav::Costmap make_open_map() {
    foray::nav::Costmap map;
    std::string error;
    map.init(kW, kH, kRes, foray::nav::Vec2{0.0, 0.0}, &error);
    for (int iy = 0; iy < kH; ++iy) {
        for (int ix = 0; ix < kW; ++ix) {
            map.set_cost(ix, iy, foray::nav::kCostFree);
        }
    }
    return map;
}

/// 用"指令速度 × dt"推演，直到到达或超步数。
struct DriveResult {
    foray::nav::NavStatus status{foray::nav::NavStatus::kIdle};
    double final_distance{0.0};
    int steps{0};
    foray::nav::NavError error{foray::nav::NavError::kNone};
};

DriveResult drive(foray::nav::NavEngine* engine, const foray::nav::CostmapView& map,
                  const foray::nav::Pose2& start, const foray::nav::Pose2& goal,
                  int max_steps = 4000) {
    DriveResult result;
    foray::nav::NavGoal nav_goal;
    nav_goal.pose = goal;
    result.error = engine->set_goal(map, nav_goal, start);
    if (result.error != foray::nav::NavError::kNone) {
        result.status = foray::nav::NavStatus::kFailed;
        return result;
    }
    foray::nav::Pose2 pose = start;
    int step = 0;
    for (; step < max_steps; ++step) {
        foray::nav::VelocityCommand command{};
        const foray::nav::NavError step_error = engine->step(map, pose, kDt, &command);
        if (engine->status() == foray::nav::NavStatus::kFailed) {
            result.error = step_error;
            break;
        }
        pose.x += command.vx * kDt;
        pose.y += command.vy * kDt;
        pose.yaw += command.omega * kDt;
        if (engine->status() == foray::nav::NavStatus::kReached) {
            break;
        }
    }
    result.steps = step;
    result.status = engine->status();
    result.final_distance = std::hypot(pose.x - goal.x, pose.y - goal.y);
    return result;
}

/// 走一条直线，必须真的到终点。
void test_reaches_goal() {
    const foray::nav::Costmap map = make_open_map();
    foray::nav::NavEngine engine;
    const DriveResult r =
        drive(&engine, map, foray::nav::Pose2{0.5, 0.5, 0.0}, foray::nav::Pose2{3.5, 2.5, 0.0});
    check(r.status == foray::nav::NavStatus::kReached, "engine: reaches the goal");
    check(r.final_distance <= 0.15, "engine: final distance within tolerance");
    check(engine.replan_count() == 0, "engine: no replan needed on a clear straight run");
}

/// 目标在致死格里：规划就该失败，且状态是 kFailed。
void test_goal_in_lethal_fails() {
    foray::nav::Costmap map = make_open_map();
    map.set_cost(40, 30, foray::nav::kCostLethal);
    map.set_cost(41, 30, foray::nav::kCostLethal);
    map.set_cost(40, 31, foray::nav::kCostLethal);
    map.set_cost(41, 31, foray::nav::kCostLethal);

    foray::nav::NavEngine engine;
    foray::nav::NavGoal goal;
    goal.pose = foray::nav::Pose2{2.025, 1.525, 0.0}; // 正好落在 (40,30) 格心
    const foray::nav::NavError error = engine.set_goal(map, goal, foray::nav::Pose2{0.5, 0.5, 0.0});
    check(error != foray::nav::NavError::kNone, "engine: lethal goal rejected");
    check(engine.status() == foray::nav::NavStatus::kFailed, "engine: status becomes failed");
}

/// 被强行挪到远离路径的地方，必须触发重规划并仍然到点。
void test_deviation_triggers_replan() {
    const foray::nav::Costmap map = make_open_map();
    foray::nav::NavEngine engine;
    foray::nav::NavGoal goal;
    goal.pose = foray::nav::Pose2{3.5, 0.5, 0.0};
    check(engine.set_goal(map, goal, foray::nav::Pose2{0.5, 0.5, 0.0}) ==
              foray::nav::NavError::kNone,
          "deviation: plan ok");

    // 先正常走几步，再把位姿"瞬移"到离路径 1.5 m 的地方（模拟被人推开）。
    foray::nav::Pose2 pose{0.5, 0.5, 0.0};
    for (int i = 0; i < 10; ++i) {
        foray::nav::VelocityCommand command{};
        engine.step(map, pose, kDt, &command);
        pose.x += command.vx * kDt;
        pose.y += command.vy * kDt;
    }
    pose.y += 1.5;

    foray::nav::VelocityCommand command{};
    engine.step(map, pose, kDt, &command);
    check(engine.replan_count() >= 1, "deviation: replanned after being pushed away");

    bool arrived = false;
    for (int i = 0; i < 4000; ++i) {
        foray::nav::VelocityCommand step_command{};
        if (engine.step(map, pose, kDt, &step_command) != foray::nav::NavError::kNone &&
            engine.status() == foray::nav::NavStatus::kFailed) {
            break;
        }
        pose.x += step_command.vx * kDt;
        pose.y += step_command.vy * kDt;
        if (engine.status() == foray::nav::NavStatus::kReached) {
            arrived = true;
            break;
        }
    }
    check(arrived, "deviation: still reaches the goal after replanning");
}

/// 没设目标就 step：必须报非法输入，不能瞎走。
void test_step_without_goal() {
    const foray::nav::Costmap map = make_open_map();
    foray::nav::NavEngine engine;
    foray::nav::VelocityCommand command{1.0, 1.0, 0.0};
    const foray::nav::NavError error =
        engine.step(map, foray::nav::Pose2{0.5, 0.5, 0.0}, kDt, &command);
    check(error == foray::nav::NavError::kInvalidInput, "engine: step before goal is invalid");
    check(command.vx == 0.0 && command.vy == 0.0, "engine: invalid step outputs zero");
}

/// 到达之后继续 step：保持零速、状态不跳。
void test_hold_after_reached() {
    const foray::nav::Costmap map = make_open_map();
    foray::nav::NavEngine engine;
    const DriveResult r =
        drive(&engine, map, foray::nav::Pose2{0.5, 0.5, 0.0}, foray::nav::Pose2{1.5, 0.5, 0.0});
    check(r.status == foray::nav::NavStatus::kReached, "hold: reached first");
    foray::nav::VelocityCommand command{1.0, 1.0, 0.0};
    engine.step(map, foray::nav::Pose2{1.5, 0.5, 0.0}, kDt, &command);
    check(command.vx == 0.0 && command.vy == 0.0, "hold: zero command after reached");
    check(engine.status() == foray::nav::NavStatus::kReached, "hold: status stays reached");
}

/// 过窄口：全程不许压到致死格，且最近距离要够（防切角撞墙）。
void test_gap_passage_keeps_clearance() {
    foray::nav::Costmap map;
    std::string error;
    check(map.init(kW, kH, kRes, foray::nav::Vec2{0.0, 0.0}, &error), "gap: init");
    for (int iy = 0; iy < kH; ++iy) {
        for (int ix = 0; ix < kW; ++ix) {
            map.set_cost(ix, iy, foray::nav::kCostFree);
        }
    }
    // 竖墙 x∈[1.9,2.1]，y<1.6 是墙，上面留 1.4 m 的口
    for (int iy = 0; iy < kH; ++iy) {
        for (int ix = 0; ix < kW; ++ix) {
            const double x = 0.025 + 0.05 * ix;
            const double y = 0.025 + 0.05 * iy;
            if (x >= 1.9 && x <= 2.1 && y < 1.6) {
                map.set_cost(ix, iy, foray::nav::kCostLethal);
            }
        }
    }

    foray::nav::NavEngine engine;
    foray::nav::NavGoal goal;
    goal.pose = foray::nav::Pose2{3.4, 1.0, 0.0};
    check(engine.set_goal(map, goal, foray::nav::Pose2{0.5, 0.5, 0.0}) ==
              foray::nav::NavError::kNone,
          "gap: plan ok");

    foray::nav::Pose2 pose{0.5, 0.5, 0.0};
    int hits = 0;
    double min_clear = 1e9;
    for (int step = 0; step < 4000; ++step) {
        foray::nav::VelocityCommand command{};
        engine.step(map, pose, kDt, &command);
        pose.x += command.vx * kDt;
        pose.y += command.vy * kDt;
        pose.yaw += command.omega * kDt;
        const int ix = static_cast<int>(std::floor(pose.x / kRes));
        const int iy = static_cast<int>(std::floor(pose.y / kRes));
        if (map.cost_at(ix, iy) == foray::nav::kCostLethal) {
            ++hits;
        }
        for (int dy = -8; dy <= 8; ++dy) {
            for (int dx = -8; dx <= 8; ++dx) {
                if (map.cost_at(ix + dx, iy + dy) == foray::nav::kCostLethal) {
                    const double cx = 0.025 + kRes * (ix + dx);
                    const double cy = 0.025 + kRes * (iy + dy);
                    min_clear = std::min(min_clear, std::hypot(pose.x - cx, pose.y - cy));
                }
            }
        }
        if (engine.status() == foray::nav::NavStatus::kReached) {
            break;
        }
    }
    check(engine.status() == foray::nav::NavStatus::kReached, "gap: reaches through the gap");
    check(hits == 0, "gap: never steps onto a lethal cell");
    check(min_clear >= 0.18, "gap: keeps at least 0.18 m clearance");
}

/// 目标容差比控制器停车容差还小时，也必须真的到得了（原来会卡在"几乎到"）。
void test_tight_goal_tolerance() {
    const foray::nav::Costmap map = make_open_map();
    foray::nav::NavEngine engine;
    foray::nav::NavGoal goal;
    goal.pose = foray::nav::Pose2{2.5, 1.5, 0.0};
    goal.xy_tolerance = 0.08;

    foray::nav::Pose2 pose{0.5, 0.5, 0.0};
    check(engine.set_goal(map, goal, pose) == foray::nav::NavError::kNone, "tight: plan ok");
    bool arrived = false;
    for (int step = 0; step < 4000; ++step) {
        foray::nav::VelocityCommand command{};
        if (engine.step(map, pose, kDt, &command) != foray::nav::NavError::kNone &&
            engine.status() == foray::nav::NavStatus::kFailed) {
            break;
        }
        pose.x += command.vx * kDt;
        pose.y += command.vy * kDt;
        if (engine.status() == foray::nav::NavStatus::kReached) {
            arrived = true;
            break;
        }
    }
    check(arrived, "tight: reaches with an 8 cm tolerance");
}

/// reset() 不能把构造时给的参数丢掉（否则行为会被悄悄改回默认值）。
void test_reset_keeps_params() {
    foray::nav::AstarParams planner_params;
    planner_params.smooth = false; // 关掉平滑：路径点数明显更多，能当指纹用
    foray::nav::NavEngine engine(planner_params, foray::nav::PursuitParams{});

    const foray::nav::Costmap map = make_open_map();
    foray::nav::NavGoal goal;
    goal.pose = foray::nav::Pose2{3.5, 2.5, 0.0};
    check(engine.set_goal(map, goal, foray::nav::Pose2{0.5, 0.5, 0.0}) ==
              foray::nav::NavError::kNone,
          "reset: first plan ok");
    const size_t first_points = engine.path().size();

    engine.reset();
    check(engine.set_goal(map, goal, foray::nav::Pose2{0.5, 0.5, 0.0}) ==
              foray::nav::NavError::kNone,
          "reset: second plan ok");
    check(engine.path().size() == first_points, "reset: planner params survive reset");
}

/// 非法输入不许谎报成功：dt<=0、NaN 位姿都要返回 kInvalidInput。
void test_rejects_invalid_input() {
    const foray::nav::Costmap map = make_open_map();
    foray::nav::NavEngine engine;
    foray::nav::NavGoal goal;
    goal.pose = foray::nav::Pose2{2.0, 1.0, 0.0};
    check(engine.set_goal(map, goal, foray::nav::Pose2{0.5, 0.5, 0.0}) ==
              foray::nav::NavError::kNone,
          "invalid: plan ok");

    foray::nav::VelocityCommand command{};
    check(engine.step(map, foray::nav::Pose2{0.5, 0.5, 0.0}, 0.0, &command) ==
              foray::nav::NavError::kInvalidInput,
          "invalid: dt == 0 rejected");
    check(command.vx == 0.0 && command.vy == 0.0, "invalid: zero output on bad dt");
    check(engine.status() == foray::nav::NavStatus::kFollowing, "invalid: state not corrupted");

    const double nan = std::nan("");
    check(engine.step(map, foray::nav::Pose2{nan, 0.5, 0.0}, kDt, &command) ==
              foray::nav::NavError::kInvalidInput,
          "invalid: NaN pose rejected");
    check(engine.status() != foray::nav::NavStatus::kReached, "invalid: NaN pose is not 'reached'");
}

} // namespace

int main() {
    test_reaches_goal();
    test_goal_in_lethal_fails();
    test_deviation_triggers_replan();
    test_step_without_goal();
    test_hold_after_reached();
    test_gap_passage_keeps_clearance();
    test_tight_goal_tolerance();
    test_reset_keeps_params();
    test_rejects_invalid_input();

    if (g_failures == 0) {
        std::printf("ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
}
