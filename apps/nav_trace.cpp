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

// 导航诊断：把每一帧的位姿、指令、净空、状态打出来。
//
// 用途：改控制环之前先有这个。只看"测试红/绿"是猜；看逐帧数据才知道错在哪一步。
//
// 用法：
//   nav_trace --yaml maps/rmul_2027/rmul_2027.yaml --sx 1.9 --sy 5.1 --gx 6.6 --gy 0.5
//   加 --csv 输出成 CSV（方便画图）
//   加 --stall-only 只在"卡住"（连续 N 帧零速）时打印上下文

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "foray/nav/grid_map.hpp"
#include "foray/nav/nav_engine.hpp"
#include "foray/nav/params.hpp"

namespace {

constexpr double kDt = 0.05;

/// 每格到最近致死格的距离（倒角近似）。诊断用，精度够。
std::vector<float> make_clearance(const foray::nav::CostmapView& map) {
    const int w = map.width();
    const int h = map.height();
    const double res = map.resolution();
    const float kInf = 1e9f;
    const float step = static_cast<float>(res);
    const float diag = static_cast<float>(std::sqrt(2.0) * res);
    std::vector<float> field(static_cast<size_t>(w) * static_cast<size_t>(h), kInf);
    auto at = [w](int x, int y) {
        return static_cast<size_t>(y) * static_cast<size_t>(w) + static_cast<size_t>(x);
    };
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (map.cost_at(x, y) == foray::nav::kCostLethal) {
                field[at(x, y)] = 0.0f;
            }
        }
    }
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float& v = field[at(x, y)];
            if (x > 0)
                v = std::min(v, field[at(x - 1, y)] + step);
            if (y > 0)
                v = std::min(v, field[at(x, y - 1)] + step);
            if (x > 0 && y > 0)
                v = std::min(v, field[at(x - 1, y - 1)] + diag);
            if (x + 1 < w && y > 0)
                v = std::min(v, field[at(x + 1, y - 1)] + diag);
        }
    }
    for (int y = h - 1; y >= 0; --y) {
        for (int x = w - 1; x >= 0; --x) {
            float& v = field[at(x, y)];
            if (x + 1 < w)
                v = std::min(v, field[at(x + 1, y)] + step);
            if (y + 1 < h)
                v = std::min(v, field[at(x, y + 1)] + step);
            if (x + 1 < w && y + 1 < h)
                v = std::min(v, field[at(x + 1, y + 1)] + diag);
            if (x > 0 && y + 1 < h)
                v = std::min(v, field[at(x - 1, y + 1)] + diag);
        }
    }
    return field;
}

float clearance_at(const foray::nav::CostmapView& map, const std::vector<float>& field,
                   const foray::nav::Vec2& p) {
    const int x = static_cast<int>(std::floor((p.x - map.origin().x) / map.resolution()));
    const int y = static_cast<int>(std::floor((p.y - map.origin().y) / map.resolution()));
    if (x < 0 || y < 0 || x >= map.width() || y >= map.height()) {
        return 0.0f;
    }
    return field[static_cast<size_t>(y) * static_cast<size_t>(map.width()) +
                 static_cast<size_t>(x)];
}

void usage() {
    std::printf("usage: nav_trace --yaml FILE --sx X --sy Y --gx X --gy Y [--csv FILE] "
                "[--stall-only] [--params YAML] [--robot YAML] [--regions FILE]\n");
}

} // namespace

int main(int argc, char** argv) {
    std::string yaml;
    double sx = 0.5;
    double sy = 0.5;
    double gx = 3.4;
    double gy = 1.0;
    std::string csv;
    bool stall_only = false;
    std::string params_path;
    std::string robot_path;
    std::string regions_path;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool has = (i + 1 < argc);
        if (a == "--yaml" && has) {
            yaml = argv[++i];
        } else if (a == "--sx" && has) {
            sx = std::atof(argv[++i]);
        } else if (a == "--sy" && has) {
            sy = std::atof(argv[++i]);
        } else if (a == "--gx" && has) {
            gx = std::atof(argv[++i]);
        } else if (a == "--gy" && has) {
            gy = std::atof(argv[++i]);
        } else if (a == "--csv" && has) {
            csv = argv[++i];
        } else if (a == "--params" && has) {
            params_path = argv[++i];
        } else if (a == "--robot" && has) {
            robot_path = argv[++i];
        } else if (a == "--regions" && has) {
            regions_path = argv[++i];
        } else if (a == "--stall-only") {
            stall_only = true;
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
    const std::vector<float> field = make_clearance(map);

    foray::nav::NavParams params;
    if (!params_path.empty()) {
        std::string params_error;
        if (!foray::nav::load_params(params_path, robot_path, &params, &params_error)) {
            std::printf("参数加载失败: %s\n", params_error.c_str());
            return 1;
        }
    } else if (!robot_path.empty()) {
        std::string params_error;
        if (!foray::nav::load_params_file(robot_path, &params, &params_error)) {
            std::printf("参数加载失败: %s\n", params_error.c_str());
            return 1;
        }
    }

    foray::nav::NavEngine engine(params.astar, params.pursuit);
    engine.options() = params.engine;
    if (!regions_path.empty()) {
        std::vector<foray::nav::RectRegion> regions;
        std::string regions_error;
        if (!foray::nav::load_regions_file(regions_path, &regions, &regions_error)) {
            std::printf("区域表加载失败: %s\n", regions_error.c_str());
            return 1;
        }
        engine.set_preferred_regions(regions);
        std::printf("区域表: %zu 块（隧道标为偏好）\n", regions.size());
    }
    foray::nav::NavGoal goal;
    goal.pose = foray::nav::Pose2{gx, gy, 0.0};
    foray::nav::Pose2 pose{sx, sy, 0.0};
    const foray::nav::NavError plan_error = engine.set_goal(map, goal, pose);
    if (plan_error != foray::nav::NavError::kNone) {
        std::printf("规划失败 err=%d\n", static_cast<int>(plan_error));
        return 1;
    }
    std::printf("path points=%zu\n", engine.path().size());

    std::FILE* out = nullptr;
    if (!csv.empty()) {
        out = std::fopen(csv.c_str(), "w");
        if (out != nullptr) {
            std::fprintf(out, "t,x,y,vx,vy,clearance,status,replans\n");
        }
    }

    int zero_streak = 0;
    int frame = 0;
    for (; frame < 4000; ++frame) {
        foray::nav::VelocityCommand command{};
        const foray::nav::NavError err = engine.step(map, pose, kDt, &command);
        const float clear = clearance_at(map, field, foray::nav::Vec2{pose.x, pose.y});
        const double speed = std::hypot(command.vx, command.vy);
        if (speed < 1e-6) {
            ++zero_streak;
        } else {
            zero_streak = 0;
        }

        if (out != nullptr) {
            std::fprintf(out, "%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d\n", frame * kDt, pose.x, pose.y,
                         command.vx, command.vy, clear, static_cast<int>(engine.status()),
                         engine.replan_count());
        }
        const bool interesting = (!stall_only) || zero_streak == 5 || engine.replan_count() > 0;
        if (interesting) {
            std::printf("t=%5.2f pos(%6.2f,%5.2f) v(%5.2f,%5.2f) clear=%5.2f st=%d rep=%d%s\n",
                        frame * kDt, pose.x, pose.y, command.vx, command.vy, clear,
                        static_cast<int>(engine.status()), engine.replan_count(),
                        zero_streak >= 5 ? "  <-- stalled" : "");
        }
        if (engine.status() == foray::nav::NavStatus::kFailed) {
            std::printf("FAILED err=%d at frame %d\n", static_cast<int>(err), frame);
            break;
        }
        pose.x += command.vx * kDt;
        pose.y += command.vy * kDt;
        pose.yaw += command.omega * kDt;
        if (engine.status() == foray::nav::NavStatus::kReached) {
            std::printf("REACHED at frame %d (%.2f s)\n", frame, frame * kDt);
            break;
        }
    }
    if (out != nullptr) {
        std::fclose(out);
    }
    return 0;
}
