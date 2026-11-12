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

// Costmap 的独立测试：分层顺序、膨胀衰减、未知不被刷通、动态障碍占格。

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "foray/nav/costmap.hpp"
#include "foray/nav/grid_map.hpp"

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
    if (!condition) {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

foray::nav::MapMeta make_meta() {
    foray::nav::MapMeta meta;
    meta.resolution = 0.05;
    meta.origin_x = 0.0;
    meta.origin_y = 0.0;
    return meta;
}

/// 造一张全白（自由）的 40x40 P5 图，分辨率 0.05 => 2m x 2m。
std::vector<uint8_t> make_free_pgm(int width, int height) {
    const std::string header =
        "P5\n" + std::to_string(width) + " " + std::to_string(height) + "\n255\n";
    std::vector<uint8_t> bytes(header.begin(), header.end());
    bytes.insert(bytes.end(), static_cast<size_t>(width) * static_cast<size_t>(height), 255);
    return bytes;
}

void test_static_layer() {
    foray::nav::GridMap static_map;
    std::string error;
    check(static_map.load_from_pgm_bytes(make_free_pgm(40, 40), make_meta(), &error),
          "static map loads");

    foray::nav::Costmap costmap;
    check(costmap.init(40, 40, 0.05, foray::nav::Vec2{0.0, 0.0}, &error), "costmap init");
    check(costmap.set_static_layer(static_map, &error), "static layer set");
    check(costmap.cost_at(10, 10) == foray::nav::kCostFree, "free cell stays free");
    check(costmap.lethal_count() == 0, "no lethal yet");

    // 尺寸不一致必须失败
    foray::nav::Costmap small;
    check(small.init(10, 10, 0.05, foray::nav::Vec2{}, &error), "small init");
    check(!small.set_static_layer(static_map, &error), "size mismatch rejected");
}

void test_forbidden_polygon() {
    foray::nav::Costmap costmap;
    std::string error;
    check(costmap.init(40, 40, 0.05, foray::nav::Vec2{0.0, 0.0}, &error), "init");
    // 覆盖 [0.5,1.0] x [0.5,1.0] 的方块（米）
    const std::vector<foray::nav::Vec2> square = {
        {0.5, 0.5},
        {1.0, 0.5},
        {1.0, 1.0},
        {0.5, 1.0},
    };
    costmap.fill_polygon(square);
    int ix = -1;
    int iy = -1;
    check(costmap.world_to_grid(foray::nav::Vec2{0.75, 0.75}, &ix, &iy), "inside maps to grid");
    check(costmap.cost_at(ix, iy) == foray::nav::kCostLethal, "polygon interior is lethal");
    check(costmap.world_to_grid(foray::nav::Vec2{1.5, 1.5}, &ix, &iy), "outside maps to grid");
    check(costmap.cost_at(ix, iy) == foray::nav::kCostUnknown,
          "outside polygon untouched (unknown default)");
}

void test_inflation() {
    foray::nav::Costmap costmap;
    std::string error;
    // 全自由，再点一格致死
    check(costmap.init(60, 60, 0.05, foray::nav::Vec2{0.0, 0.0}, &error), "init");
    for (int iy = 0; iy < 60; ++iy) {
        for (int ix = 0; ix < 60; ++ix) {
            costmap.set_cost(ix, iy, foray::nav::kCostFree);
        }
    }
    costmap.set_cost(30, 30, foray::nav::kCostLethal);
    costmap.inflate(0.10, 0.40, 4.0); // 内切 2 格，膨胀 8 格

    check(costmap.cost_at(30, 30) == foray::nav::kCostLethal, "lethal stays lethal");
    // 距离 0.05m（1 格）在 inscribed 内 => 致死
    check(costmap.cost_at(31, 30) == foray::nav::kCostLethal, "inside inscribed becomes lethal");
    // 距离 0.20m（4 格）在衰减带内 => 非零且低于致死
    const uint8_t mid = costmap.cost_at(34, 30);
    check(mid > 0 && mid < foray::nav::kCostLethal, "decay band gives gradient cost");
    // 距离 0.45m（9 格）超出膨胀半径 => 不加代价
    check(costmap.cost_at(39, 30) == foray::nav::kCostFree, "beyond inflation radius untouched");
}

void test_unknown_not_inflated_through() {
    foray::nav::Costmap costmap;
    std::string error;
    check(costmap.init(20, 20, 0.05, foray::nav::Vec2{0.0, 0.0}, &error), "init");
    // 默认全是未知；只在中间放一格致死
    costmap.set_cost(10, 10, foray::nav::kCostLethal);
    costmap.inflate(0.05, 0.30, 4.0);
    check(costmap.cost_at(11, 10) == foray::nav::kCostUnknown,
          "unknown is never rewritten by inflation");
}

void test_dynamic_obstacles() {
    foray::nav::Costmap costmap;
    std::string error;
    check(costmap.init(40, 40, 0.05, foray::nav::Vec2{0.0, 0.0}, &error), "init");
    for (int iy = 0; iy < 40; ++iy) {
        for (int ix = 0; ix < 40; ++ix) {
            costmap.set_cost(ix, iy, foray::nav::kCostFree);
        }
    }
    std::vector<foray::nav::DynamicObstacle> obstacles;
    foray::nav::DynamicObstacle obstacle;
    obstacle.center = foray::nav::Vec2{1.0, 1.0};
    obstacle.radius = 0.15;
    obstacles.push_back(obstacle);
    costmap.stamp_obstacles(obstacles);

    int ix = -1;
    int iy = -1;
    check(costmap.world_to_grid(foray::nav::Vec2{1.0, 1.0}, &ix, &iy), "obstacle center in grid");
    check(costmap.cost_at(ix, iy) == foray::nav::kCostLethal, "obstacle center is lethal");
    check(costmap.world_to_grid(foray::nav::Vec2{1.6, 1.6}, &ix, &iy), "far point in grid");
    check(costmap.cost_at(ix, iy) == foray::nav::kCostFree, "far from obstacle stays free");
}

// ---- 2026-11-03 审计 CM-1 的回归测试 ----
/// `inflate()` 必须幂等：连调多次，可通行区不许越缩越窄。
void test_inflate_is_idempotent() {
    foray::nav::Costmap costmap;
    std::string error;
    check(costmap.init(100, 100, 0.05, foray::nav::Vec2{0.0, 0.0}, &error), "idem: init");
    for (int iy = 0; iy < 100; ++iy) {
        for (int ix = 0; ix < 100; ++ix) {
            costmap.set_cost(ix, iy, foray::nav::kCostFree);
        }
    }
    // 中间放一小块障碍
    for (int iy = 48; iy < 52; ++iy) {
        for (int ix = 48; ix < 52; ++ix) {
            costmap.set_cost(ix, iy, foray::nav::kCostLethal);
        }
    }
    const int before = costmap.lethal_count();
    costmap.inflate(0.20, 0.50, 3.0);
    const int after_first = costmap.lethal_count();
    for (int k = 0; k < 3; ++k) {
        costmap.inflate(0.20, 0.50, 3.0);
    }
    const int after_fourth = costmap.lethal_count();

    check(after_first > before, "idem: inflation actually happens");
    check(after_first == after_fourth, "idem: repeated inflate does not shrink free space");
}

// ---- 2026-11-03 审计 CM-2 / CM-5 的回归测试 ----

/// CM-2：`cost_scaling` 越大，离障碍同样距离的代价应当**越低**（衰减越快）。
void test_cost_scaling_decays_faster_when_larger() {
    auto cost_at_distance = [](double scaling) {
        foray::nav::Costmap costmap;
        std::string error;
        costmap.init(100, 100, 0.05, foray::nav::Vec2{0.0, 0.0}, &error);
        for (int iy = 0; iy < 100; ++iy) {
            for (int ix = 0; ix < 100; ++ix) {
                costmap.set_cost(ix, iy, foray::nav::kCostFree);
            }
        }
        for (int iy = 0; iy < 100; ++iy) {
            for (int ix = 0; ix < 100; ++ix) {
                if (ix < 20) {
                    costmap.set_cost(ix, iy, foray::nav::kCostLethal);
                }
            }
        }
        costmap.inflate(0.20, 0.80, scaling);
        return static_cast<int>(costmap.cost_at(30, 50)); // 离致死区约 0.5 m
    };
    const int slow = cost_at_distance(1.0);
    const int fast = cost_at_distance(8.0);
    check(fast < slow, "cost_scaling: larger factor decays faster (cheaper)");
}

/// CM-5：未知格不参与距离变换时，贴边界的路线和穿中央一样便宜。
void test_unknown_contributes_to_inflation() {
    foray::nav::Costmap costmap;
    std::string error;
    check(costmap.init(60, 60, 0.05, foray::nav::Vec2{0.0, 0.0}, &error), "unknown: init");
    // 左半边全未知，右半边全自由
    for (int iy = 0; iy < 60; ++iy) {
        for (int ix = 0; ix < 60; ++ix) {
            costmap.set_cost(ix, iy, ix < 30 ? foray::nav::kCostUnknown : foray::nav::kCostFree);
        }
    }
    costmap.inflate(0.05, 0.50, 3.0);
    // 紧贴未知边界那一格应当被抬价，而不是保持自由价
    check(costmap.cost_at(31, 30) > foray::nav::kCostFree,
          "unknown: cells next to unknown are inflated");
    check(costmap.cost_at(29, 30) == foray::nav::kCostUnknown,
          "unknown: unknown cells stay unknown");
}

// ---- 2026-11-03 审计 CM-3 与「细窄禁行带」的回归测试 ----

/// CM-3：带过期时间戳的障碍要按"不存在"处理，不许永远留在图上。
void test_stale_obstacles_expire() {
    foray::nav::Costmap costmap;
    std::string error;
    check(costmap.init(40, 40, 0.05, foray::nav::Vec2{0.0, 0.0}, &error), "stale: init");
    for (int iy = 0; iy < 40; ++iy) {
        for (int ix = 0; ix < 40; ++ix) {
            costmap.set_cost(ix, iy, foray::nav::kCostFree);
        }
    }
    costmap.set_obstacle_timeout_ns(100000000); // 100 ms

    foray::nav::DynamicObstacle old_obstacle;
    old_obstacle.center = foray::nav::Vec2{1.0, 1.0};
    old_obstacle.radius = 0.15;
    old_obstacle.stamp_ns = 1; // 早得离谱，必然过期
    costmap.stamp_obstacles({old_obstacle});

    int ix = -1;
    int iy = -1;
    check(costmap.world_to_grid(foray::nav::Vec2{1.0, 1.0}, &ix, &iy), "stale: center in grid");
    check(costmap.cost_at(ix, iy) != foray::nav::kCostLethal, "stale: expired obstacle is ignored");
}

/// 比一格还窄的禁行带也要画出来，不能整条漏掉。
void test_narrow_polygon_is_rasterized() {
    foray::nav::Costmap costmap;
    std::string error;
    check(costmap.init(80, 80, 0.05, foray::nav::Vec2{0.0, 0.0}, &error), "narrow: init");
    for (int iy = 0; iy < 80; ++iy) {
        for (int ix = 0; ix < 80; ++ix) {
            costmap.set_cost(ix, iy, foray::nav::kCostFree);
        }
    }
    // 2 cm 宽（0.02 m < 0.05 m 一格）的横贯带
    std::vector<foray::nav::Vec2> band = {{0.0, 1.00}, {4.0, 1.00}, {4.0, 1.02}, {0.0, 1.02}};
    costmap.fill_polygon(band, foray::nav::kCostLethal);
    check(costmap.lethal_count() > 0, "narrow: sub-cell band still marks cells");
}

/// CM-8：包围盒裁剪不能把"贴着地图边"的障碍或禁行区裁掉。
void test_bbox_clipping_keeps_edges() {
    foray::nav::Costmap costmap;
    std::string error;
    check(costmap.init(40, 40, 0.05, foray::nav::Vec2{0.0, 0.0}, &error), "bbox: init");
    for (int iy = 0; iy < 40; ++iy) {
        for (int ix = 0; ix < 40; ++ix) {
            costmap.set_cost(ix, iy, foray::nav::kCostFree);
        }
    }
    // 障碍中心正好在左上角
    foray::nav::DynamicObstacle obstacle;
    obstacle.center = foray::nav::Vec2{0.0, 0.0};
    obstacle.radius = 0.1;
    costmap.stamp_obstacles({obstacle});
    check(costmap.cost_at(0, 0) == foray::nav::kCostLethal, "bbox: corner obstacle still marks");

    // 一半在地图外的禁行多边形
    std::vector<foray::nav::Vec2> poly = {{-0.5, 1.0}, {0.5, 1.0}, {0.5, 1.2}, {-0.5, 1.2}};
    costmap.fill_polygon(poly, foray::nav::kCostLethal);
    int ix = -1;
    int iy = -1;
    check(costmap.world_to_grid(foray::nav::Vec2{0.2, 1.1}, &ix, &iy), "bbox: point in grid");
    check(costmap.cost_at(ix, iy) == foray::nav::kCostLethal, "bbox: clipped polygon still marks");
}

} // namespace

int main() {
    test_static_layer();
    test_forbidden_polygon();
    test_inflation();
    test_unknown_not_inflated_through();
    test_dynamic_obstacles();
    test_inflate_is_idempotent();
    test_cost_scaling_decays_faster_when_larger();
    test_unknown_contributes_to_inflation();
    test_stale_obstacles_expire();
    test_narrow_polygon_is_rasterized();
    test_bbox_clipping_keeps_edges();

    if (g_failures == 0) {
        std::printf("ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
}
