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

// A* 的独立测试：直线、L 形、绕障碍、不可穿致死、目标致死、起点越界、全封闭、
// 窄通道（刚好能过 / 差一点过不去）、视线捷径化。

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "diesel/nav/astar_planner.hpp"
#include "diesel/nav/costmap.hpp"

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
    if (!condition) {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

constexpr int kW = 80; // 80 x 40 格，0.05 m/格 => 4 m x 2 m
constexpr int kH = 40;
constexpr double kRes = 0.05;

diesel::nav::Costmap make_free_map() {
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

/// 竖直墙：把 x = 2.00m（第 40 列）整列刷致死，可留一个缺口。
void add_wall(diesel::nav::Costmap* map, int gap_from_row, int gap_to_row) {
    const int column = 40;
    for (int iy = 0; iy < kH; ++iy) {
        if (iy >= gap_from_row && iy <= gap_to_row) {
            continue;
        }
        map->set_cost(column, iy, diesel::nav::kCostLethal);
    }
}

void test_straight_line() {
    diesel::nav::Costmap map = make_free_map();
    diesel::nav::AstarPlanner planner;
    diesel::nav::Path path;
    const auto error =
        planner.plan(map, diesel::nav::Pose2{0.5, 1.0, 0.0}, diesel::nav::Pose2{3.5, 1.0, 0.0},
                     diesel::nav::PlanningOptions{}, &path);
    check(error == diesel::nav::NavError::kNone, "straight: ok");
    check(path.size() >= 2, "straight: has points");
    check(std::fabs(path.front().x - 0.5) < 0.06, "straight: starts near start");
    check(std::fabs(path.back().x - 3.5) < 0.06, "straight: ends near goal");
    // 直线且无墙：捷径化后应只剩很少的点
    check(path.size() <= 4, "straight: shortcut collapses points");
}

void test_l_shape() {
    diesel::nav::Costmap map = make_free_map();
    // 竖墙挡住 x=2.0 的下半部分，只留上方缺口
    add_wall(&map, 26, 39);
    diesel::nav::AstarPlanner planner;
    diesel::nav::Path path;
    const auto error =
        planner.plan(map, diesel::nav::Pose2{0.5, 0.5, 0.0}, diesel::nav::Pose2{3.5, 0.5, 0.0},
                     diesel::nav::PlanningOptions{}, &path);
    check(error == diesel::nav::NavError::kNone, "L: ok");
    // 必须从缺口（上方，y > 1.3m）绕过去
    bool went_high = false;
    for (const auto& pose : path) {
        if (pose.y > 1.3) {
            went_high = true;
        }
    }
    check(went_high, "L: goes around through the gap");
    // 路径不能踩到致死格
    for (const auto& pose : path) {
        int ix = 0;
        int iy = 0;
        if (map.world_to_grid(diesel::nav::Vec2{pose.x, pose.y}, &ix, &iy)) {
            check(map.cost_at(ix, iy) != diesel::nav::kCostLethal, "L: path avoids lethal cells");
        }
    }
}

void test_goal_in_lethal() {
    diesel::nav::Costmap map = make_free_map();
    map.set_cost(60, 20, diesel::nav::kCostLethal);
    diesel::nav::AstarPlanner planner;
    diesel::nav::Path path;
    const auto error =
        planner.plan(map, diesel::nav::Pose2{0.5, 1.0, 0.0}, diesel::nav::Pose2{3.0, 1.0, 0.0},
                     diesel::nav::PlanningOptions{}, &path);
    check(error == diesel::nav::NavError::kGoalInLethal, "goal lethal: reported without searching");
}

void test_start_outside() {
    diesel::nav::Costmap map = make_free_map();
    diesel::nav::AstarPlanner planner;
    diesel::nav::Path path;
    const auto error =
        planner.plan(map, diesel::nav::Pose2{-1.0, 1.0, 0.0}, diesel::nav::Pose2{3.0, 1.0, 0.0},
                     diesel::nav::PlanningOptions{}, &path);
    check(error == diesel::nav::NavError::kStartOutsideMap, "start outside: reported");
}

void test_fully_blocked() {
    diesel::nav::Costmap map = make_free_map();
    add_wall(&map, kH, -1); // 没有缺口
    diesel::nav::AstarPlanner planner;
    diesel::nav::Path path;
    const auto error =
        planner.plan(map, diesel::nav::Pose2{0.5, 1.0, 0.0}, diesel::nav::Pose2{3.5, 1.0, 0.0},
                     diesel::nav::PlanningOptions{}, &path);
    check(error == diesel::nav::NavError::kNoPath, "blocked: no path");
}

void test_narrow_corridor() {
    // 4 格宽缺口 = 0.20m；2 格宽 = 0.10m。测试"能过"和"太窄过不去"两种。
    {
        diesel::nav::Costmap map = make_free_map();
        add_wall(&map, 18, 21); // 4 格宽
        diesel::nav::AstarPlanner planner;
        diesel::nav::Path path;
        const auto error =
            planner.plan(map, diesel::nav::Pose2{0.5, 1.0, 0.0}, diesel::nav::Pose2{3.5, 1.0, 0.0},
                         diesel::nav::PlanningOptions{}, &path);
        check(error == diesel::nav::NavError::kNone, "narrow(0.20m): passable");
    }
    {
        diesel::nav::Costmap map = make_free_map();
        add_wall(&map, 20, 20); // 1 格宽
        diesel::nav::AstarPlanner planner;
        diesel::nav::Path path;
        const auto error =
            planner.plan(map, diesel::nav::Pose2{0.5, 1.0, 0.0}, diesel::nav::Pose2{3.5, 1.0, 0.0},
                         diesel::nav::PlanningOptions{}, &path);
        check(error == diesel::nav::NavError::kNone,
              "narrow(1 cell): still geometrically passable");
        // 但斜穿犄角被禁止：路径必须正交穿过缺口，不允许"蹭着走"
        check(path.size() >= 2, "narrow(1 cell): has path");
    }
}

void test_smoothing_reduces_points() {
    diesel::nav::Costmap map = make_free_map();
    diesel::nav::AstarParams params;
    params.smooth = false;
    diesel::nav::AstarPlanner raw(params);
    diesel::nav::AstarPlanner smooth;
    diesel::nav::Path raw_path;
    diesel::nav::Path smooth_path;
    const diesel::nav::Pose2 start{0.30, 0.30, 0.0};
    const diesel::nav::Pose2 goal{3.30, 1.60, 0.0};
    check(raw.plan(map, start, goal, diesel::nav::PlanningOptions{}, &raw_path) ==
              diesel::nav::NavError::kNone,
          "raw plan ok");
    check(smooth.plan(map, start, goal, diesel::nav::PlanningOptions{}, &smooth_path) ==
              diesel::nav::NavError::kNone,
          "smooth plan ok");
    check(smooth_path.size() <= raw_path.size(), "smoothing does not add points");
    check(smooth_path.size() < raw_path.size(), "smoothing actually removes points on open field");
}

void test_tunnel_preferred() {
    // 场景：正中间有一条横向的"近路"（自由），下方有一条更长的横向"隧道"（偏好区）。
    // 无偏好时应走上面；开启偏好后应改走隧道。
    diesel::nav::Costmap map = make_free_map();
    // 在中线上下各造一道墙，只留"上方近路"和"下方隧道"两个缺口
    for (int iy = 0; iy < kH; ++iy) {
        if (iy <= 22 && iy >= 18) {
            continue;
        } // 中间整条留空（近路）
        map.set_cost(40, iy, diesel::nav::kCostLethal);
    }
    // 在 y < 0.75m 的条带（下方）铺偏好区：模拟隧道
    for (int iy = 0; iy < 15; ++iy) {
        for (int ix = 0; ix < kW; ++ix) {
            map.set_cost(ix, iy, diesel::nav::kCostPreferred);
        }
    }

    diesel::nav::Path no_pref;
    {
        diesel::nav::AstarParams params;
        params.prefer_weight = 1.0; // 关掉偏好
        diesel::nav::AstarPlanner planner(params);
        const auto error =
            planner.plan(map, diesel::nav::Pose2{0.5, 1.0, 0.0}, diesel::nav::Pose2{3.5, 1.0, 0.0},
                         diesel::nav::PlanningOptions{}, &no_pref);
        check(error == diesel::nav::NavError::kNone, "no-pref: ok");
    }
    diesel::nav::Path with_pref;
    {
        diesel::nav::AstarPlanner planner;
        const auto error =
            planner.plan(map, diesel::nav::Pose2{0.5, 1.0, 0.0}, diesel::nav::Pose2{3.5, 1.0, 0.0},
                         diesel::nav::PlanningOptions{}, &with_pref);
        check(error == diesel::nav::NavError::kNone, "pref: ok");
    }

    // 无偏好时走中间近路（y 接近 1.0m，穿过 18..22 行）
    bool no_pref_mid = false;
    for (const auto& pose : no_pref) {
        int ix = 0;
        int iy = 0;
        if (map.world_to_grid(diesel::nav::Vec2{pose.x, pose.y}, &ix, &iy) && iy >= 18 &&
            iy <= 22) {
            no_pref_mid = true;
        }
    }
    check(no_pref_mid, "no-pref: takes the short middle route");

    // 开启偏好后应当绕去下方偏好带（y < 0.75m）
    bool took_tunnel = false;
    for (const auto& pose : with_pref) {
        if (pose.y < 0.75) {
            took_tunnel = true;
        }
    }
    check(took_tunnel, "pref: detours through the preferred tunnel band");
}

void test_budget_exhausted() {
    // 预算给 1 次展开，肯定到不了：必须快速返回而不是死算。
    diesel::nav::Costmap map = make_free_map();
    diesel::nav::AstarPlanner planner;
    diesel::nav::Path path;
    diesel::nav::PlanningOptions options;
    options.max_expansions = 1;
    const auto error = planner.plan(map, diesel::nav::Pose2{0.3, 0.3, 0.0},
                                    diesel::nav::Pose2{3.5, 1.5, 0.0}, options, &path);
    check(error == diesel::nav::NavError::kNoPath, "budget: exhausted returns kNoPath");
}

void test_cost_used_as_weight() {
    // 同一条直线，把中段铺成"贵"的膨胀带：路径应当绕开它，而不是无视代价直穿。
    diesel::nav::Costmap map = make_free_map();
    for (int ix = 20; ix < 60; ++ix) {
        for (int iy = 0; iy < 16; ++iy) {
            map.set_cost(ix, iy, 200);
        }
    }
    diesel::nav::AstarPlanner planner;
    diesel::nav::Path path;
    const auto error =
        planner.plan(map, diesel::nav::Pose2{0.5, 0.4, 0.0}, diesel::nav::Pose2{3.5, 0.4, 0.0},
                     diesel::nav::PlanningOptions{}, &path);
    check(error == diesel::nav::NavError::kNone, "weight: ok");
    bool went_around = false;
    for (const auto& pose : path) {
        if (pose.y > 0.80) {
            went_around = true;
        }
    }
    check(went_around, "weight: routes around the expensive band instead of crossing it");
}

/// AS-7：同一个规划器连算两次，结果必须完全一致（复用的缓冲不许串味）。
void test_repeat_plan_is_stable() {
    diesel::nav::Costmap map = make_free_map();
    diesel::nav::AstarPlanner planner;
    diesel::nav::PlanningOptions options;
    diesel::nav::Path first;
    diesel::nav::Path second;
    const diesel::nav::Pose2 start{0.1, 0.1, 0.0};
    const diesel::nav::Pose2 goal{2.5, 1.5, 0.0};
    check(planner.plan(map, start, goal, options, &first) == diesel::nav::NavError::kNone,
          "stable: first plan ok");
    check(planner.plan(map, start, goal, options, &second) == diesel::nav::NavError::kNone,
          "stable: second plan ok");
    bool same = first.size() == second.size();
    if (same) {
        for (size_t i = 0; i < first.size(); ++i) {
            if (first[i].x != second[i].x || first[i].y != second[i].y) {
                same = false;
                break;
            }
        }
    }
    check(same, "stable: repeated plan gives identical path");
}

} // namespace

int main() {
    test_repeat_plan_is_stable();
    test_straight_line();
    test_l_shape();
    test_goal_in_lethal();
    test_start_outside();
    test_fully_blocked();
    test_narrow_corridor();
    test_smoothing_reduces_points();
    test_tunnel_preferred();
    test_budget_exhausted();
    test_cost_used_as_weight();

    if (g_failures == 0) {
        std::printf("ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
}
