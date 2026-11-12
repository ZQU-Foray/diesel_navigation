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

// 随机压测：在真实地图上随机撒起点 / 终点，批量跑「规划 → 跟踪 → 到达」，
// 统计到达率、压墙帧数、里程、用时、重规划次数。
//
// 为什么要有它：PR 里写「105 组全部到达」这种数字，**没有可复现的入口就等于没有证据**。
// 这里用固定种子的 std::mt19937，同一台机器上谁跑都是同一批任务。
//
// 用法：
//   nav_fuzz --yaml MAP.yaml [--count N] [--seed S] [--clearance M]
//            [--params YAML] [--robot YAML] [--regions FILE] [--csv OUT]
//
// 位姿积分用「指令速度 × dt」，和 nav_demo 一样——只为验证闭环逻辑，不代表实车动力学。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "foray/nav/costmap.hpp"
#include "foray/nav/grid_map.hpp"
#include "foray/nav/nav_engine.hpp"
#include "foray/nav/params.hpp"
#include "foray/nav/regions.hpp"

namespace {

constexpr double kDt = 0.05;    // 20 Hz
constexpr int kMaxSteps = 4000; // 单任务 200 s 上限

void usage() {
    std::printf("usage: nav_fuzz --yaml MAP.yaml [--count N] [--seed S] [--clearance M]\n"
                "                [--params YAML] [--robot YAML] [--regions FILE] [--csv OUT]\n"
                "  --count     任务数，默认 100\n"
                "  --seed      随机种子（固定即可复现），默认 2027\n"
                "  --clearance 起终点要求的最小净空（米），默认 0.30\n"
                "  --csv       逐任务结果 CSV\n");
}

/// 某点在原图上的净空（到最近致死格的近似距离，米）；用 20 格窗口扫。
double local_clearance(const foray::nav::CostmapView& map, const foray::nav::Vec2& p,
                       int window_cells) {
    int cx = 0;
    int cy = 0;
    if (!map.world_to_grid(p, &cx, &cy)) {
        return 0.0;
    }
    double best = 1e9;
    for (int dy = -window_cells; dy <= window_cells; ++dy) {
        for (int dx = -window_cells; dx <= window_cells; ++dx) {
            if (map.cost_at(cx + dx, cy + dy) == foray::nav::kCostLethal) {
                best = std::min(best, std::hypot(dx, dy) * map.resolution());
            }
        }
    }
    return best;
}

bool in_regions(const std::vector<foray::nav::RectRegion>& regions, const foray::nav::Vec2& p) {
    for (const auto& r : regions) {
        if (r.contains(p)) {
            return true;
        }
    }
    return false;
}

} // namespace

int main(int argc, char** argv) {
    std::string yaml;
    int count = 100;
    unsigned int seed = 2027;
    double clearance_needed = 0.30;
    std::string params_path;
    std::string robot_path;
    std::string regions_path;
    std::string csv_path;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool has = (i + 1 < argc);
        if (a == "--help" || a == "-h") {
            usage();
            return 0;
        } else if (a == "--yaml" && has) {
            yaml = argv[++i];
        } else if (a == "--count" && has) {
            count = std::atoi(argv[++i]);
        } else if (a == "--seed" && has) {
            seed = static_cast<unsigned int>(std::strtoul(argv[++i], nullptr, 10));
        } else if (a == "--clearance" && has) {
            clearance_needed = std::atof(argv[++i]);
        } else if (a == "--params" && has) {
            params_path = argv[++i];
        } else if (a == "--robot" && has) {
            robot_path = argv[++i];
        } else if (a == "--regions" && has) {
            regions_path = argv[++i];
        } else if (a == "--csv" && has) {
            csv_path = argv[++i];
        } else {
            usage();
            return 2;
        }
    }
    if (yaml.empty()) {
        usage();
        return 2;
    }

    foray::nav::GridMap map;
    std::string error;
    if (!map.load_from_yaml(yaml, &error)) {
        std::printf("地图加载失败: %s\n", error.c_str());
        return 1;
    }

    foray::nav::NavParams params;
    if (!params_path.empty()) {
        if (!foray::nav::load_params(params_path, robot_path, &params, &error)) {
            std::printf("参数加载失败: %s\n", error.c_str());
            return 1;
        }
    } else if (!robot_path.empty()) {
        if (!foray::nav::load_params_file(robot_path, &params, &error)) {
            std::printf("参数加载失败: %s\n", error.c_str());
            return 1;
        }
    }

    std::vector<foray::nav::RectRegion> regions;
    if (!regions_path.empty() && !foray::nav::load_regions_file(regions_path, &regions, &error)) {
        std::printf("区域表加载失败: %s\n", error.c_str());
        return 1;
    }

    // 候选点：整张图上每隔 2 格取一个「净空足够」的格子。
    const int window =
        std::max(1, static_cast<int>(std::ceil(clearance_needed / map.resolution())));
    std::vector<foray::nav::Pose2> candidates;
    for (int iy = 2; iy < map.height() - 2; iy += 2) {
        for (int ix = 2; ix < map.width() - 2; ix += 2) {
            foray::nav::Vec2 world{};
            if (!map.grid_to_world(ix, iy, &world)) {
                continue;
            }
            if (local_clearance(map, world, window) >= clearance_needed) {
                candidates.push_back(foray::nav::Pose2{world.x, world.y, 0.0});
            }
        }
    }
    if (candidates.size() < 2) {
        std::printf("可用起点太少（%zu 个），把 --clearance 调小一点\n", candidates.size());
        return 1;
    }

    std::mt19937 rng(seed);
    std::uniform_int_distribution<size_t> pick(0, candidates.size() - 1);

    std::FILE* csv = nullptr;
    if (!csv_path.empty()) {
        csv = std::fopen(csv_path.c_str(), "w");
        if (csv != nullptr) {
            std::fprintf(csv,
                         "task,reached,steps,seconds,distance,min_clearance,wall_frames,replans\n");
        }
    }

    int reached = 0;
    long long wall_frames = 0;
    double total_distance = 0.0;
    double total_seconds = 0.0;
    double worst_clearance = 1e9;

    for (int task = 0; task < count; ++task) {
        const foray::nav::Pose2 start = candidates[pick(rng)];
        foray::nav::Pose2 goal = candidates[pick(rng)];
        // 太近的任务没有意义（原地就到了），重抽一次
        if (std::hypot(start.x - goal.x, start.y - goal.y) < 1.0) {
            goal = candidates[pick(rng)];
        }

        foray::nav::NavEngine engine(params.astar, params.pursuit);
        engine.options() = params.engine;
        engine.set_preferred_regions(regions);

        foray::nav::NavGoal nav_goal;
        nav_goal.pose = goal;
        const foray::nav::NavError plan_error = engine.set_goal(map, nav_goal, start);
        if (plan_error != foray::nav::NavError::kNone) {
            if (csv != nullptr) {
                std::fprintf(csv, "%d,0,0,0.00,0.00,0.00,0,0\n", task);
            }
            continue;
        }

        foray::nav::Pose2 pose = start;
        double distance = 0.0;
        double min_clearance = 1e9;
        int wall = 0;
        int step = 0;
        for (; step < kMaxSteps; ++step) {
            foray::nav::VelocityCommand command{};
            const foray::nav::NavError err = engine.step(map, pose, kDt, &command);
            if (err != foray::nav::NavError::kNone &&
                engine.status() == foray::nav::NavStatus::kFailed) {
                break;
            }
            // 压墙：车心落在原图的致死格上；隧道区域里不算（那是人标过的通道）。
            const foray::nav::Vec2 here{pose.x, pose.y};
            if (map.cost_at(static_cast<int>((here.x - map.origin().x) / map.resolution()),
                            static_cast<int>((here.y - map.origin().y) / map.resolution())) ==
                    foray::nav::kCostLethal &&
                !in_regions(regions, here)) {
                ++wall;
            }
            if (!in_regions(regions, here)) {
                min_clearance = std::min(min_clearance, local_clearance(map, here, 12));
            }
            const double v = std::hypot(command.vx, command.vy);
            distance += v * kDt;
            pose.x += command.vx * kDt;
            pose.y += command.vy * kDt;
            if (engine.status() == foray::nav::NavStatus::kReached) {
                ++reached;
                break;
            }
        }
        if (min_clearance < 1e9) {
            worst_clearance = std::min(worst_clearance, min_clearance);
        }
        wall_frames += wall;
        total_distance += distance;
        total_seconds += step * kDt;
        if (csv != nullptr) {
            std::fprintf(csv, "%d,%d,%d,%.2f,%.2f,%.2f,%d,%d\n", task,
                         engine.status() == foray::nav::NavStatus::kReached ? 1 : 0, step,
                         step * kDt, distance, min_clearance < 1e9 ? min_clearance : 0.0, wall,
                         engine.replan_count());
        }
    }
    if (csv != nullptr) {
        std::fclose(csv);
    }

    std::printf("任务 %d 个（种子 %u，净空门槛 %.2f m）：到达 %d，压墙帧 %lld，"
                "平均里程 %.2f m，平均用时 %.2f s，最差净空 %.3f m\n",
                count, seed, clearance_needed, reached, wall_frames,
                count > 0 ? total_distance / count : 0.0, count > 0 ? total_seconds / count : 0.0,
                worst_clearance < 1e9 ? worst_clearance : 0.0);
    return reached == count ? 0 : 1;
}
