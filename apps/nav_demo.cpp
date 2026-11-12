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

// 导航演示：不依赖 ROS，用一张人造地图把「规划 → 跟踪 → 到达」跑一遍。
//
// 用法：
//   nav_demo                 默认地图（4 m x 3 m，中间一堵墙，留一个口）
//   nav_demo --pgm a.pgm --yaml a.yaml --goal-x 3 --goal-y 2   读真实地图
//
// 位姿积分用最简单的「指令速度 × dt」，只为演示闭环，不代表实车动力学。

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "diesel/nav/costmap.hpp"
#include "diesel/nav/grid_map.hpp"
#include "diesel/nav/nav_engine.hpp"
#include "diesel/nav/params.hpp"

namespace {

constexpr double kDt = 0.05;    // 20 Hz
constexpr int kMaxSteps = 4000; // 200 s 上限
constexpr int kPrintEvery = 40; // 每 2 s 打一行

/// 造一张 4 m × 3 m、0.05 m/格 的地图：中间一堵竖墙，靠上留一个 1.4 m 的口。
/// 口只有 0.5 m 时，车体半径 0.2 m 的车根本过不去——这是真实约束，不是 bug。
diesel::nav::Costmap make_demo_map() {
    constexpr int kW = 80;
    constexpr int kH = 60;
    diesel::nav::Costmap map;
    std::string error;
    map.init(kW, kH, 0.05, diesel::nav::Vec2{0.0, 0.0}, &error);
    for (int iy = 0; iy < kH; ++iy) {
        for (int ix = 0; ix < kW; ++ix) {
            map.set_cost(ix, iy, diesel::nav::kCostFree);
        }
    }
    // 竖墙：x ∈ [1.9, 2.1] m，只在 y ∈ [0, 2.2) 内是墙，其余留口
    for (int iy = 0; iy < kH; ++iy) {
        for (int ix = 0; ix < kW; ++ix) {
            const double x = 0.025 + 0.05 * ix;
            const double y = 0.025 + 0.05 * iy;
            if (x >= 1.9 && x <= 2.1 && y < 1.6) {
                map.set_cost(ix, iy, diesel::nav::kCostLethal);
            }
        }
    }
    return map;
}

void print_usage() {
    std::printf(
        "用法: nav_demo [--yaml FILE] [--goal-x X] [--goal-y Y] [--params YAML] [--robot YAML]\n"
        "               [--regions FILE]\n"
        "  默认用内置演示地图（4 m x 3 m，中间一堵墙留口）。\n"
        "  --params   通用参数文件（如 config/shared.yaml）\n"
        "  --robot    车种覆盖文件（如 config/rmul_2027.yaml），叠加在 --params 之上\n"
        "  --regions  区域表（隧道 / 通道，标成偏好）\n");
}

} // namespace

int main(int argc, char** argv) {
    std::string yaml_path;
    double goal_x = 3.4;
    double goal_y = 1.0;
    std::string params_path;
    std::string robot_path;
    std::string regions_path;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool has_value = (i + 1 < argc);
        if (arg == "--help" || arg == "-h") {
            print_usage();
            return 0;
        } else if (arg == "--yaml" && has_value) {
            yaml_path = argv[++i];
        } else if (arg == "--goal-x" && has_value) {
            goal_x = std::atof(argv[++i]);
        } else if (arg == "--goal-y" && has_value) {
            goal_y = std::atof(argv[++i]);
        } else if (arg == "--params" && has_value) {
            params_path = argv[++i];
        } else if (arg == "--robot" && has_value) {
            robot_path = argv[++i];
        } else if (arg == "--regions" && has_value) {
            regions_path = argv[++i];
        } else {
            std::printf("未知参数: %s\n", arg.c_str());
            print_usage();
            return 2;
        }
    }

    diesel::nav::GridMap loaded;
    diesel::nav::Costmap demo;
    const diesel::nav::CostmapView* map = nullptr;

    if (!yaml_path.empty()) {
        std::string error;
        if (!loaded.load_from_yaml(yaml_path, &error)) {
            std::printf("地图加载失败: %s\n", error.c_str());
            return 1;
        }
        map = &loaded;
    } else {
        demo = make_demo_map();
        map = &demo;
    }

    // 参数：命令行给了就读文件，没给就用编译期默认值（与解析器默认保持一致）。
    diesel::nav::NavParams params;
    if (!params_path.empty()) {
        std::string params_error;
        if (!diesel::nav::load_params(params_path, robot_path, &params, &params_error)) {
            std::printf("参数加载失败: %s\n", params_error.c_str());
            return 1;
        }
    } else if (!robot_path.empty()) {
        std::string params_error;
        if (!diesel::nav::load_params_file(robot_path, &params, &params_error)) {
            std::printf("参数加载失败: %s\n", params_error.c_str());
            return 1;
        }
    }

    diesel::nav::NavEngine engine(params.astar, params.pursuit);
    engine.options() = params.engine;
    if (!regions_path.empty()) {
        std::vector<diesel::nav::RectRegion> regions;
        std::string regions_error;
        if (!diesel::nav::load_regions_file(regions_path, &regions, &regions_error)) {
            std::printf("区域表加载失败: %s\n", regions_error.c_str());
            return 1;
        }
        engine.set_preferred_regions(regions);
    }
    if (!params_path.empty() || !robot_path.empty()) {
        std::printf("参数：车体半径 %.2f m，限速 %.2f m/s，最多重规划 %d 次（来源 %s%s%s）\n",
                    params.engine.robot_radius, params.pursuit.max_linear,
                    params.engine.max_replans, params_path.c_str(), robot_path.empty() ? "" : " + ",
                    robot_path.c_str());
    }
    const diesel::nav::Pose2 start{0.5, 0.5, 0.0};
    diesel::nav::NavGoal goal;
    goal.pose = diesel::nav::Pose2{goal_x, goal_y, 0.0};

    const diesel::nav::NavError plan_error = engine.set_goal(*map, goal, start);
    if (plan_error != diesel::nav::NavError::kNone) {
        std::printf("规划失败，原因码 %d\n", static_cast<int>(plan_error));
        return 1;
    }
    std::printf("地图 %.2f x %.2f m，起点 (%.2f, %.2f)，目标 (%.2f, %.2f)，路径 %zu 点\n",
                map->width() * map->resolution(), map->height() * map->resolution(), start.x,
                start.y, goal.pose.x, goal.pose.y, engine.path().size());

    diesel::nav::Pose2 pose = start;
    double travelled = 0.0;
    int step = 0;
    for (; step < kMaxSteps; ++step) {
        diesel::nav::VelocityCommand command{};
        const diesel::nav::NavError step_error = engine.step(*map, pose, kDt, &command);
        if (step_error != diesel::nav::NavError::kNone &&
            engine.status() == diesel::nav::NavStatus::kFailed) {
            std::printf("失败，原因码 %d（第 %d 步）\n", static_cast<int>(step_error), step);
            break;
        }
        const double vx = command.vx;
        const double vy = command.vy;
        pose.x += vx * kDt;
        pose.y += vy * kDt;
        pose.yaw += command.omega * kDt;
        travelled += std::hypot(vx, vy) * kDt;

        if (step % kPrintEvery == 0) {
            std::printf("t=%5.2fs  位置 (%5.2f, %5.2f)  速度 (%5.2f, %5.2f)  重规划 %d\n",
                        step * kDt, pose.x, pose.y, vx, vy, engine.replan_count());
        }
        if (engine.status() == diesel::nav::NavStatus::kReached) {
            break;
        }
    }

    const double final_distance = std::hypot(pose.x - goal.pose.x, pose.y - goal.pose.y);
    std::printf("\n结果：%s\n",
                engine.status() == diesel::nav::NavStatus::kReached ? "到达" : "未到达");
    std::printf("  步数 %d（%.2f s），走过 %.2f m，终点误差 %.3f m，重规划 %d 次\n", step,
                step * kDt, travelled, final_distance, engine.replan_count());
    return engine.status() == diesel::nav::NavStatus::kReached ? 0 : 1;
}
