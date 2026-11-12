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

#include "diesel/nav/costmap.hpp"
#include "diesel/nav/nav_engine.hpp"

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

diesel::nav::Costmap make_open_map() {
    diesel::nav::Costmap map;
    std::string error;
    map.init(kW, kH, kRes, diesel::nav::Vec2{0.0, 0.0}, &error);
    for (int iy = 0; iy < kH; ++iy) {
        for (int ix = 0; ix < kW; ++ix) {
            map.set_cost(ix, iy, diesel::nav::kCostFree);
        }
    }
    return map;
}

/// 用"指令速度 × dt"推演，直到到达或超步数。
struct DriveResult {
    diesel::nav::NavStatus status{diesel::nav::NavStatus::kIdle};
    double final_distance{0.0};
    int steps{0};
    diesel::nav::NavError error{diesel::nav::NavError::kNone};
};

DriveResult drive(diesel::nav::NavEngine* engine, const diesel::nav::CostmapView& map,
                  const diesel::nav::Pose2& start, const diesel::nav::Pose2& goal,
                  int max_steps = 4000) {
    DriveResult result;
    diesel::nav::NavGoal nav_goal;
    nav_goal.pose = goal;
    result.error = engine->set_goal(map, nav_goal, start);
    if (result.error != diesel::nav::NavError::kNone) {
        result.status = diesel::nav::NavStatus::kFailed;
        return result;
    }
    diesel::nav::Pose2 pose = start;
    int step = 0;
    for (; step < max_steps; ++step) {
        diesel::nav::VelocityCommand command{};
        const diesel::nav::NavError step_error = engine->step(map, pose, kDt, &command);
        if (engine->status() == diesel::nav::NavStatus::kFailed) {
            result.error = step_error;
            break;
        }
        pose.x += command.vx * kDt;
        pose.y += command.vy * kDt;
        pose.yaw += command.omega * kDt;
        if (engine->status() == diesel::nav::NavStatus::kReached) {
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
    const diesel::nav::Costmap map = make_open_map();
    diesel::nav::NavEngine engine;
    const DriveResult r =
        drive(&engine, map, diesel::nav::Pose2{0.5, 0.5, 0.0}, diesel::nav::Pose2{3.5, 2.5, 0.0});
    check(r.status == diesel::nav::NavStatus::kReached, "engine: reaches the goal");
    check(r.final_distance <= 0.15, "engine: final distance within tolerance");
    check(engine.replan_count() == 0, "engine: no replan needed on a clear straight run");
}

/// 目标在致死格里：规划就该失败，且状态是 kFailed。
void test_goal_in_lethal_fails() {
    diesel::nav::Costmap map = make_open_map();
    map.set_cost(40, 30, diesel::nav::kCostLethal);
    map.set_cost(41, 30, diesel::nav::kCostLethal);
    map.set_cost(40, 31, diesel::nav::kCostLethal);
    map.set_cost(41, 31, diesel::nav::kCostLethal);

    diesel::nav::NavEngine engine;
    diesel::nav::NavGoal goal;
    goal.pose = diesel::nav::Pose2{2.025, 1.525, 0.0}; // 正好落在 (40,30) 格心
    const diesel::nav::NavError error =
        engine.set_goal(map, goal, diesel::nav::Pose2{0.5, 0.5, 0.0});
    check(error != diesel::nav::NavError::kNone, "engine: lethal goal rejected");
    check(engine.status() == diesel::nav::NavStatus::kFailed, "engine: status becomes failed");
}

/// 被强行挪到远离路径的地方，必须触发重规划并仍然到点。
void test_deviation_triggers_replan() {
    const diesel::nav::Costmap map = make_open_map();
    diesel::nav::NavEngine engine;
    diesel::nav::NavGoal goal;
    goal.pose = diesel::nav::Pose2{3.5, 0.5, 0.0};
    check(engine.set_goal(map, goal, diesel::nav::Pose2{0.5, 0.5, 0.0}) ==
              diesel::nav::NavError::kNone,
          "deviation: plan ok");

    // 先正常走几步，再把位姿"瞬移"到离路径 1.5 m 的地方（模拟被人推开）。
    diesel::nav::Pose2 pose{0.5, 0.5, 0.0};
    for (int i = 0; i < 10; ++i) {
        diesel::nav::VelocityCommand command{};
        engine.step(map, pose, kDt, &command);
        pose.x += command.vx * kDt;
        pose.y += command.vy * kDt;
    }
    pose.y += 1.5;

    diesel::nav::VelocityCommand command{};
    engine.step(map, pose, kDt, &command);
    check(engine.replan_count() >= 1, "deviation: replanned after being pushed away");

    bool arrived = false;
    for (int i = 0; i < 4000; ++i) {
        diesel::nav::VelocityCommand step_command{};
        if (engine.step(map, pose, kDt, &step_command) != diesel::nav::NavError::kNone &&
            engine.status() == diesel::nav::NavStatus::kFailed) {
            break;
        }
        pose.x += step_command.vx * kDt;
        pose.y += step_command.vy * kDt;
        if (engine.status() == diesel::nav::NavStatus::kReached) {
            arrived = true;
            break;
        }
    }
    check(arrived, "deviation: still reaches the goal after replanning");
}

/// 没设目标就 step：必须报非法输入，不能瞎走。
void test_step_without_goal() {
    const diesel::nav::Costmap map = make_open_map();
    diesel::nav::NavEngine engine;
    diesel::nav::VelocityCommand command{1.0, 1.0, 0.0};
    const diesel::nav::NavError error =
        engine.step(map, diesel::nav::Pose2{0.5, 0.5, 0.0}, kDt, &command);
    check(error == diesel::nav::NavError::kInvalidInput, "engine: step before goal is invalid");
    check(command.vx == 0.0 && command.vy == 0.0, "engine: invalid step outputs zero");
}

/// 到达之后继续 step：保持零速、状态不跳。
void test_hold_after_reached() {
    const diesel::nav::Costmap map = make_open_map();
    diesel::nav::NavEngine engine;
    const DriveResult r =
        drive(&engine, map, diesel::nav::Pose2{0.5, 0.5, 0.0}, diesel::nav::Pose2{1.5, 0.5, 0.0});
    check(r.status == diesel::nav::NavStatus::kReached, "hold: reached first");
    diesel::nav::VelocityCommand command{1.0, 1.0, 0.0};
    engine.step(map, diesel::nav::Pose2{1.5, 0.5, 0.0}, kDt, &command);
    check(command.vx == 0.0 && command.vy == 0.0, "hold: zero command after reached");
    check(engine.status() == diesel::nav::NavStatus::kReached, "hold: status stays reached");
}

/// 过窄口：全程不许压到致死格，且最近距离要够（防切角撞墙）。
void test_gap_passage_keeps_clearance() {
    diesel::nav::Costmap map;
    std::string error;
    check(map.init(kW, kH, kRes, diesel::nav::Vec2{0.0, 0.0}, &error), "gap: init");
    for (int iy = 0; iy < kH; ++iy) {
        for (int ix = 0; ix < kW; ++ix) {
            map.set_cost(ix, iy, diesel::nav::kCostFree);
        }
    }
    // 竖墙 x∈[1.9,2.1]，y<1.6 是墙，上面留 1.4 m 的口
    for (int iy = 0; iy < kH; ++iy) {
        for (int ix = 0; ix < kW; ++ix) {
            const double x = 0.025 + 0.05 * ix;
            const double y = 0.025 + 0.05 * iy;
            if (x >= 1.9 && x <= 2.1 && y < 1.6) {
                map.set_cost(ix, iy, diesel::nav::kCostLethal);
            }
        }
    }

    diesel::nav::NavEngine engine;
    diesel::nav::NavGoal goal;
    goal.pose = diesel::nav::Pose2{3.4, 1.0, 0.0};
    check(engine.set_goal(map, goal, diesel::nav::Pose2{0.5, 0.5, 0.0}) ==
              diesel::nav::NavError::kNone,
          "gap: plan ok");

    diesel::nav::Pose2 pose{0.5, 0.5, 0.0};
    int hits = 0;
    double min_clear = 1e9;
    for (int step = 0; step < 4000; ++step) {
        diesel::nav::VelocityCommand command{};
        engine.step(map, pose, kDt, &command);
        pose.x += command.vx * kDt;
        pose.y += command.vy * kDt;
        pose.yaw += command.omega * kDt;
        const int ix = static_cast<int>(std::floor(pose.x / kRes));
        const int iy = static_cast<int>(std::floor(pose.y / kRes));
        if (map.cost_at(ix, iy) == diesel::nav::kCostLethal) {
            ++hits;
        }
        for (int dy = -8; dy <= 8; ++dy) {
            for (int dx = -8; dx <= 8; ++dx) {
                if (map.cost_at(ix + dx, iy + dy) == diesel::nav::kCostLethal) {
                    const double cx = 0.025 + kRes * (ix + dx);
                    const double cy = 0.025 + kRes * (iy + dy);
                    min_clear = std::min(min_clear, std::hypot(pose.x - cx, pose.y - cy));
                }
            }
        }
        if (engine.status() == diesel::nav::NavStatus::kReached) {
            break;
        }
    }
    check(engine.status() == diesel::nav::NavStatus::kReached, "gap: reaches through the gap");
    check(hits == 0, "gap: never steps onto a lethal cell");
    // 护栏半径是自适应的：正常 0.6×半径（0.15 m），车已经贴得比这更近时会退，
    // 但地板是 0.4×半径（0.10 m）。所以这里断言"不低于 0.4×半径"。
    check(min_clear >= 0.095, "gap: keeps a floor of 0.4 x robot radius");
}

/// 目标容差比控制器停车容差还小时，也必须真的到得了（原来会卡在"几乎到"）。
void test_tight_goal_tolerance() {
    const diesel::nav::Costmap map = make_open_map();
    diesel::nav::NavEngine engine;
    diesel::nav::NavGoal goal;
    goal.pose = diesel::nav::Pose2{2.5, 1.5, 0.0};
    goal.xy_tolerance = 0.08;

    diesel::nav::Pose2 pose{0.5, 0.5, 0.0};
    check(engine.set_goal(map, goal, pose) == diesel::nav::NavError::kNone, "tight: plan ok");
    bool arrived = false;
    for (int step = 0; step < 4000; ++step) {
        diesel::nav::VelocityCommand command{};
        if (engine.step(map, pose, kDt, &command) != diesel::nav::NavError::kNone &&
            engine.status() == diesel::nav::NavStatus::kFailed) {
            break;
        }
        pose.x += command.vx * kDt;
        pose.y += command.vy * kDt;
        if (engine.status() == diesel::nav::NavStatus::kReached) {
            arrived = true;
            break;
        }
    }
    check(arrived, "tight: reaches with an 8 cm tolerance");
}

/// reset() 不能把构造时给的参数丢掉（否则行为会被悄悄改回默认值）。
void test_reset_keeps_params() {
    diesel::nav::AstarParams planner_params;
    planner_params.smooth = false; // 关掉平滑：路径点数明显更多，能当指纹用
    diesel::nav::NavEngine engine(planner_params, diesel::nav::PursuitParams{});

    const diesel::nav::Costmap map = make_open_map();
    diesel::nav::NavGoal goal;
    goal.pose = diesel::nav::Pose2{3.5, 2.5, 0.0};
    check(engine.set_goal(map, goal, diesel::nav::Pose2{0.5, 0.5, 0.0}) ==
              diesel::nav::NavError::kNone,
          "reset: first plan ok");
    const size_t first_points = engine.path().size();

    engine.reset();
    check(engine.set_goal(map, goal, diesel::nav::Pose2{0.5, 0.5, 0.0}) ==
              diesel::nav::NavError::kNone,
          "reset: second plan ok");
    check(engine.path().size() == first_points, "reset: planner params survive reset");
}

/// 非法输入不许谎报成功：dt<=0、NaN 位姿都要返回 kInvalidInput。
void test_rejects_invalid_input() {
    const diesel::nav::Costmap map = make_open_map();
    diesel::nav::NavEngine engine;
    diesel::nav::NavGoal goal;
    goal.pose = diesel::nav::Pose2{2.0, 1.0, 0.0};
    check(engine.set_goal(map, goal, diesel::nav::Pose2{0.5, 0.5, 0.0}) ==
              diesel::nav::NavError::kNone,
          "invalid: plan ok");

    diesel::nav::VelocityCommand command{};
    check(engine.step(map, diesel::nav::Pose2{0.5, 0.5, 0.0}, 0.0, &command) ==
              diesel::nav::NavError::kInvalidInput,
          "invalid: dt == 0 rejected");
    check(command.vx == 0.0 && command.vy == 0.0, "invalid: zero output on bad dt");
    check(engine.status() == diesel::nav::NavStatus::kFollowing, "invalid: state not corrupted");

    const double nan = std::nan("");
    check(engine.step(map, diesel::nav::Pose2{nan, 0.5, 0.0}, kDt, &command) ==
              diesel::nav::NavError::kInvalidInput,
          "invalid: NaN pose rejected");
    check(engine.status() != diesel::nav::NavStatus::kReached,
          "invalid: NaN pose is not 'reached'");
}

/// 原点非零的地图：路径必须与真实的墙对齐。
/// 这个 bug 之前一直躲着，就是因为所有测试地图的 origin 都是 (0,0)
/// （2026-11-07：真图 origin=(-0.4,-0.4)，安全图用了 (0,0)，整张布局偏 0.4 m，路径穿墙）。
void test_nonzero_origin_alignment() {
    diesel::nav::Costmap map;
    std::string error;
    const diesel::nav::Vec2 origin{-2.0, -3.0};
    check(map.init(kW, kH, kRes, origin, &error), "origin: init");

    // 全自由，再在中间立一道竖墙（世界坐标 x ∈ [-0.05, 0.05]）
    for (int iy = 0; iy < kH; ++iy) {
        for (int ix = 0; ix < kW; ++ix) {
            map.set_cost(ix, iy, diesel::nav::kCostFree);
        }
    }
    for (int iy = 0; iy < kH; ++iy) {
        for (int ix = 0; ix < kW; ++ix) {
            diesel::nav::Vec2 world{};
            if (!map.grid_to_world(ix, iy, &world)) {
                continue;
            }
            if (world.x >= -0.05 && world.x <= 0.05 && world.y < -1.0) {
                map.set_cost(ix, iy, diesel::nav::kCostLethal);
            }
        }
    }

    // 起点在墙左、终点在墙右；墙从 y=-3 立到 y=-1，只有上方能绕。
    diesel::nav::NavEngine engine;
    diesel::nav::NavGoal goal;
    goal.pose = diesel::nav::Pose2{1.0, -2.0, 0.0};
    const diesel::nav::Pose2 start{-1.0, -2.0, 0.0};
    check(engine.set_goal(map, goal, start) == diesel::nav::NavError::kNone, "origin: plan ok");

    // 逐段采样，确认没有一步踩在致死格上。
    int crossings = 0;
    const auto& path = engine.path();
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const double length = std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
        const int samples = std::max(2, static_cast<int>(length / (kRes * 0.5)));
        for (int k = 0; k <= samples; ++k) {
            const double t = static_cast<double>(k) / samples;
            const diesel::nav::Vec2 p{path[i].x + (path[i + 1].x - path[i].x) * t,
                                      path[i].y + (path[i + 1].y - path[i].y) * t};
            int ix = 0;
            int iy = 0;
            if (!map.world_to_grid(p, &ix, &iy)) {
                continue;
            }
            if (map.cost_at(ix, iy) == diesel::nav::kCostLethal) {
                ++crossings;
            }
        }
    }
    check(crossings == 0, "origin: path never crosses a wall on a shifted map");
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
    test_nonzero_origin_alignment();

    if (g_failures == 0) {
        std::printf("ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
}
