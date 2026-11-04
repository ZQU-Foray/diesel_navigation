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

#include "foray/nav/astar_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "foray/nav/costmap_view.hpp"

namespace foray {
namespace nav {
namespace {

struct Node {
    double f{0.0};
    int index{0};
};

struct NodeGreater {
    bool operator()(const Node& a, const Node& b) const {
        return a.f > b.f;
    }
};

/// 膨胀层可能写到的最大代价（与 costmap.cpp 的 kMaxInflatedCost 对齐）。
constexpr uint8_t kMaxInflatedCostRef = 252;

/// 捷径化时允许"直连比沿格链略贵"的余量（2%）。
/// 来历：格子化会把直线走成锯齿，格链代价总是略高于真直线；不留余量就永远捷径化不了。
/// 2% 是 2026-10-28 在 240x160 全自由图上试出来的上限——再大就会为了省两格而绕远。
constexpr double kSmoothingCostSlack = 1.02;

/// 8 邻接的行走代价权重：膨胀格更"贵"，但不禁止（禁止靠致死格）。
double step_weight(uint8_t cost, double cost_weight, double prefer_weight) {
    if (cost == kCostLethal) {
        return std::numeric_limits<double>::infinity();
    }
    if (cost == kCostUnknown) {
        return std::numeric_limits<double>::infinity();
    }
    if (cost == kCostPreferred) {
        return std::max(0.05, prefer_weight); // >1 表示"比自由格更贵"，不再被钳成 1（审计 AS-4）
    }
    if (cost == kCostFree) {
        return 1.0;
    }
    // 2..252：膨胀衰减带，线性折算成"更贵"
    const double normalized = static_cast<double>(cost - kCostFree) /
                              static_cast<double>(kMaxInflatedCostRef - kCostFree);
    return 1.0 + cost_weight * std::min(1.0, normalized);
}

/// 八方向距离（octile）启发式，单位米。
double heuristic(int dx, int dy, double resolution) {
    const int ax = std::abs(dx);
    const int ay = std::abs(dy);
    const int diagonal = std::min(ax, ay);
    const int straight = std::max(ax, ay) - diagonal;
    return (static_cast<double>(diagonal) * std::sqrt(2.0) + static_cast<double>(straight)) *
           resolution;
}

/// 保守的视线检测：沿线取足够密的采样点，任何一点不可通行就当挡。
bool line_of_sight(const CostmapView& map, const Vec2& from, const Vec2& to) {
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double length = std::sqrt(dx * dx + dy * dy);
    const double step = map.resolution() * 0.5;
    const int samples = std::max(2, static_cast<int>(std::ceil(length / std::max(1e-6, step))));
    for (int i = 0; i <= samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(samples);
        const Vec2 point{from.x + dx * t, from.y + dy * t};
        int ix = 0;
        int iy = 0;
        if (!map.world_to_grid(point, &ix, &iy)) {
            return false;
        }
        if (!map.passable(ix, iy)) {
            return false;
        }
    }
    return true;
}

/// 一段直线的行走代价（保守采样）。捷径化必须看代价，否则会把"隧道偏好"抄成直穿。
double segment_cost(const CostmapView& map, const Vec2& from, const Vec2& to, double cost_weight,
                    double prefer_weight) {
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double length = std::sqrt(dx * dx + dy * dy);
    const double step = map.resolution() * 0.5;
    const int samples = std::max(1, static_cast<int>(std::ceil(length / std::max(1e-6, step))));
    double total = 0.0;
    const double seg = length / static_cast<double>(samples);
    for (int i = 1; i <= samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(samples);
        const Vec2 point{from.x + dx * t, from.y + dy * t};
        int ix = 0;
        int iy = 0;
        if (!map.world_to_grid(point, &ix, &iy)) {
            return std::numeric_limits<double>::infinity();
        }
        const double weight = step_weight(map.cost_at(ix, iy), cost_weight, prefer_weight);
        if (!std::isfinite(weight)) {
            return std::numeric_limits<double>::infinity();
        }
        total += seg * weight;
    }
    return total;
}

} // namespace

NavError AstarPlanner::plan(const CostmapView& map, const Pose2& start, const Pose2& goal,
                            const PlanningOptions& options, Path* out) {
    if (out == nullptr) {
        return NavError::kInvalidInput;
    }
    out->clear();

    int start_x = 0;
    int start_y = 0;
    if (!map.world_to_grid(Vec2{start.x, start.y}, &start_x, &start_y)) {
        return NavError::kStartOutsideMap;
    }
    int goal_x = 0;
    int goal_y = 0;
    if (!map.world_to_grid(Vec2{goal.x, goal.y}, &goal_x, &goal_y)) {
        return NavError::kInvalidInput; // 目标在地图外：调用方给的坐标有问题
    }

    const int width = map.width();
    const int height = map.height();
    // 热路径入口：有连续内存就直接寻址，没有就退回虚函数（见 CostmapView::data）。
    const uint8_t* grid = map.data();
    const int grid_stride = map.stride();
    auto cost_of = [grid, grid_stride, &map](int gx, int gy) -> uint8_t {
        if (grid != nullptr) {
            return grid[static_cast<size_t>(gy) * static_cast<size_t>(grid_stride) +
                        static_cast<size_t>(gx)];
        }
        return map.cost_at(gx, gy);
    };
    const int max_expansions =
        options.max_expansions > 0 ? options.max_expansions : params_.max_expansions;
    const auto started = std::chrono::steady_clock::now();

    if (cost_of(start_x, start_y) == kCostLethal || cost_of(start_x, start_y) == kCostUnknown) {
        return NavError::kInvalidInput; // 起点在致死/未知格：先让上层把人放好
    }
    if (cost_of(goal_x, goal_y) == kCostLethal || cost_of(goal_x, goal_y) == kCostUnknown) {
        return NavError::kGoalInLethal;
    }

    if (width <= 0 || height <= 0) {
        return NavError::kInvalidInput;
    }
    const size_t total = static_cast<size_t>(width) * static_cast<size_t>(height);
    const double resolution = map.resolution();

    const int start_index = start_y * width + start_x;
    const int goal_index = goal_y * width + goal_x;
    if (start_index == goal_index) {
        out->push_back(Pose2{start.x, start.y, start.yaw});
        out->push_back(Pose2{goal.x, goal.y, goal.yaw});
        return NavError::kNone;
    }

    // 偏好区权重可能小于 1，启发式按最小权重缩放，保证可采纳（不破最优性）。
    const double heuristic_scale = std::min(1.0, std::max(0.05, params_.prefer_weight));

    std::vector<double> g_score(total, std::numeric_limits<double>::infinity());
    std::vector<int> parent(total, -1);
    std::vector<char> closed(total, 0);
    std::priority_queue<Node, std::vector<Node>, NodeGreater> open;

    g_score[static_cast<size_t>(start_index)] = 0.0;
    open.push(Node{heuristic_scale * heuristic(goal_x - start_x, goal_y - start_y, resolution),
                   start_index});

    const int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    int expansions = 0;
    bool found = false;
    while (!open.empty()) {
        const Node current = open.top();
        open.pop();
        if (closed[static_cast<size_t>(current.index)] != 0) {
            continue;
        }
        closed[static_cast<size_t>(current.index)] = 1;
        if (current.index == goal_index) {
            found = true;
            break;
        }
        if (++expansions > max_expansions) {
            break; // 预算耗尽：本次不证明可达，按 kNoPath 返回
        }
        if (options.deadline_ns > 0) {
            const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now() - started)
                                     .count();
            if (elapsed > options.deadline_ns) {
                break; // 超时：同上
            }
        }

        const int cx = current.index % width;
        const int cy = current.index / width;
        for (int k = 0; k < 8; ++k) {
            const int nx = cx + kDx[k];
            const int ny = cy + kDy[k];
            if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                continue;
            }
            const int neighbor = ny * width + nx;
            if (closed[static_cast<size_t>(neighbor)] != 0) {
                continue;
            }
            const uint8_t cost = cost_of(nx, ny);
            if (cost == kCostLethal || cost == kCostUnknown) {
                continue;
            }
            const bool diagonal = (kDx[k] != 0) && (kDy[k] != 0);
            if (diagonal) {
                // 禁止斜穿犄角：两条正交邻格都得能走。
                const uint8_t side_a = cost_of(nx, cy);
                const uint8_t side_b = cost_of(cx, ny);
                if (side_a == kCostLethal || side_a == kCostUnknown || side_b == kCostLethal ||
                    side_b == kCostUnknown) {
                    continue;
                }
            }
            const double step = (diagonal ? std::sqrt(2.0) : 1.0) * resolution;
            const double candidate =
                g_score[static_cast<size_t>(current.index)] +
                step * step_weight(cost, params_.cost_weight, params_.prefer_weight);
            if (candidate < g_score[static_cast<size_t>(neighbor)]) {
                g_score[static_cast<size_t>(neighbor)] = candidate;
                parent[static_cast<size_t>(neighbor)] = current.index;
                open.push(Node{candidate + heuristic_scale *
                                               heuristic(goal_x - nx, goal_y - ny, resolution),
                               neighbor});
            }
        }
    }

    if (!found) {
        return NavError::kNoPath;
    }

    // 回溯并转成世界坐标。
    std::vector<Vec2> cells;
    for (int index = goal_index; index != -1; index = parent[static_cast<size_t>(index)]) {
        const int ix = index % width;
        const int iy = index / width;
        Vec2 world{};
        if (!map.grid_to_world(ix, iy, &world)) {
            return NavError::kInvalidInput;
        }
        cells.push_back(world);
        if (index == start_index) {
            break;
        }
    }
    std::reverse(cells.begin(), cells.end());

    // 视线捷径化：能直连、且**代价不变差**才跳过中间点。
    // 只看通不通是不够的——那会把"绕隧道"抄成直穿，偏好就白设了。
    std::vector<Vec2> reduced;
    bool smoothing_aborted = false;
    if (params_.smooth && cells.size() > 2) {
        reduced.push_back(cells.front());
        size_t anchor = 0;
        while (anchor + 1 < cells.size()) {
            // 平滑也必须守预算：超时就放弃平滑、原样返回（审计 AS-1）。
            if (options.deadline_ns > 0) {
                const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::steady_clock::now() - started)
                                         .count();
                if (elapsed > options.deadline_ns) {
                    smoothing_aborted = true;
                    break;
                }
            }
            size_t best = anchor + 1;
            double best_chain = segment_cost(map, cells[anchor], cells[anchor + 1],
                                             params_.cost_weight, params_.prefer_weight);
            for (size_t next = anchor + 2; next < cells.size(); ++next) {
                best_chain += segment_cost(map, cells[next - 1], cells[next], params_.cost_weight,
                                           params_.prefer_weight);
                if (!line_of_sight(map, cells[anchor], cells[next])) {
                    continue;
                }
                const double direct = segment_cost(map, cells[anchor], cells[next],
                                                   params_.cost_weight, params_.prefer_weight);
                // 直连允许比"沿格链走"贵一点点：格子化会让直线路径的格链代价略高于直线，
                // 不留这点余量就永远捷径化不了。2% 来自实测（审计 AS-3 要求注明出处）。
                if (direct <= best_chain * kSmoothingCostSlack) {
                    best = next;
                }
            }
            reduced.push_back(cells[best]);
            anchor = best;
        }
        if (smoothing_aborted) {
            // 超预算：原样交回未平滑的格子序列，宁可路径糙，也不许超时。
            reduced = cells;
        } else if (reduced.size() >= 2 &&
                   (reduced.back().x != cells.back().x || reduced.back().y != cells.back().y)) {
            reduced.push_back(cells.back());
        }
    } else {
        reduced = cells;
    }

    out->reserve(reduced.size() + 2);
    out->push_back(Pose2{start.x, start.y, start.yaw});
    // 第一个点通常是"起点格的格心"，与真实起点最多差半格；直接接上会形成一个几厘米的折角，
    // 按弧长参数化的控制器会在首段看到零长/反向微段（审计 AS-6）。太近就跳过它。
    size_t first = 0;
    if (!reduced.empty()) {
        const double gap = std::hypot(reduced[0].x - start.x, reduced[0].y - start.y);
        if (gap < resolution) {
            first = 1;
        }
    }
    // 中间点的朝向按"前后两点连线"给：一律置 0 会让 Path 的位姿语义失效
    // （审计 AS-2：起点有朝向、中间全 0、终点又有朝向，读的人会以为朝向没意义）。
    for (size_t i = first; i < reduced.size(); ++i) {
        double yaw = 0.0;
        const Vec2 before = (i == 0) ? Vec2{start.x, start.y} : reduced[i - 1];
        const Vec2 after = (i + 1 < reduced.size()) ? reduced[i + 1] : Vec2{goal.x, goal.y};
        const double dx = after.x - before.x;
        const double dy = after.y - before.y;
        if (std::hypot(dx, dy) > 1e-9) {
            yaw = std::atan2(dy, dx);
        }
        out->push_back(Pose2{reduced[i].x, reduced[i].y, yaw});
    }
    out->push_back(Pose2{goal.x, goal.y, goal.yaw});
    return NavError::kNone;
}

} // namespace nav
} // namespace foray
