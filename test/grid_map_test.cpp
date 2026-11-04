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

// GridMap 的独立测试：不依赖任何外部框架，直接编译成可执行文件跑。
// 用例覆盖：YAML 解析、P5/P2 读入、上下翻转、三态代价、坐标换算往返、越界。

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "foray/nav/grid_map.hpp"

using foray::nav::MapMeta;

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
    if (!condition) {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

bool near(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) < eps;
}

/// 造一张 4x3 的 P5 图（PGM 里 0 = 黑 = 占据，255 = 白 = 自由）：
///   y=上  255 255 255 255
///         255 255   0 255      （黑格在图像第 1 行、第 2 列）
///   y=下  255 255 255 255
/// 占据格在图像第 2 行、第 3 列（图像坐标）；翻转后应落在栅格 iy=1。
std::vector<uint8_t> make_pgm_p5() {
    const std::string header = "P5\n4 3\n255\n";
    std::vector<uint8_t> bytes(header.begin(), header.end());
    const uint8_t pixels[12] = {
        255, 255, 255, 255, 255, 255, 0, 255, 255, 255, 255, 255,
    };
    bytes.insert(bytes.end(), pixels, pixels + 12);
    return bytes;
}

MapMeta make_meta() {
    MapMeta meta;
    meta.resolution = 0.05; // 米/格
    meta.origin_x = 1.0;
    meta.origin_y = 2.0;
    meta.negate = 0;
    meta.occupied_thresh = 0.65;
    meta.free_thresh = 0.196;
    return meta;
}

void test_yaml_parse() {
    const std::string yaml = "image: field.pgm\n"
                             "resolution: 0.05\n"
                             "origin: [-1.5, 2.5, 0.0]\n"
                             "negate: 0\n"
                             "occupied_thresh: 0.65\n"
                             "free_thresh: 0.196\n";
    MapMeta meta;
    std::string error;
    check(foray::nav::parse_map_meta(yaml, &meta, &error), "yaml parse ok");
    check(meta.image_path == "field.pgm", "yaml image");
    check(near(meta.resolution, 0.05), "yaml resolution");
    check(near(meta.origin_x, -1.5) && near(meta.origin_y, 2.5), "yaml origin");

    // 缺 resolution 必须失败
    MapMeta bad;
    check(!foray::nav::parse_map_meta("image: x.pgm\n", &bad, &error),
          "yaml without resolution fails");
    // 旋转 origin 必须失败（本仓不支持）
    MapMeta rotated;
    check(!foray::nav::parse_map_meta("image: x.pgm\nresolution: 0.05\norigin: [0,0,1.57]\n",
                                      &rotated, &error),
          "rotated origin rejected");
}

void test_p5_and_flip() {
    foray::nav::GridMap map;
    std::string error;
    check(map.load_from_pgm_bytes(make_pgm_p5(), make_meta(), &error), "p5 load ok");
    check(map.width() == 4 && map.height() == 3, "p5 size");
    check(map.loaded(), "loaded flag");

    // 图像第 1 行第 2 列是黑的；翻转后 iy = 3-1-1 = 1，ix = 2。
    check(map.cost_at(2, 1) == foray::nav::kCostLethal, "occupied cell flipped to grid row 1");
    check(map.cost_at(2, 2) == foray::nav::kCostFree, "cell above is free");
    check(map.cost_at(2, 0) == foray::nav::kCostFree, "cell below is free");
}

void test_p2_ascii() {
    const std::string text = "P2\n"
                             "# 注释行\n"
                             "3 2\n"
                             "255\n"
                             "255 255 255\n"
                             "255   0 255\n";
    std::vector<uint8_t> bytes(text.begin(), text.end());
    foray::nav::GridMap map;
    std::string error;
    MapMeta meta = make_meta();
    check(map.load_from_pgm_bytes(bytes, meta, &error), "p2 load ok");
    check(map.width() == 3 && map.height() == 2, "p2 size");
    check(map.cost_at(1, 0) == foray::nav::kCostLethal, "p2 occupied at bottom row");
}

void test_unknown_and_bounds() {
    foray::nav::GridMap map;
    std::string error;
    MapMeta meta = make_meta();
    // 127 落在两个阈值之间 => 未知（0.502）
    const std::string text = "P5\n2 1\n255\n";
    std::vector<uint8_t> bytes(text.begin(), text.end());
    bytes.push_back(127);
    bytes.push_back(255);
    check(map.load_from_pgm_bytes(bytes, meta, &error), "gray load ok");
    check(map.cost_at(0, 0) == foray::nav::kCostUnknown, "mid gray is unknown");
    check(map.cost_at(1, 0) == foray::nav::kCostFree, "white is free");
    check(map.cost_at(-1, 0) == foray::nav::kCostUnknown, "negative index unknown");
    check(map.cost_at(99, 0) == foray::nav::kCostUnknown, "out of range unknown");
    check(!map.passable(0, 0), "unknown is not passable");
}

void test_world_grid_roundtrip() {
    foray::nav::GridMap map;
    std::string error;
    check(map.load_from_pgm_bytes(make_pgm_p5(), make_meta(), &error), "roundtrip load");

    int ix = -1;
    int iy = -1;
    // origin (1.0, 2.0)、0.05 米/格：格 (0,0) 覆盖 [1.00,1.05) x [2.00,2.05)
    check(map.world_to_grid(foray::nav::Vec2{1.01, 2.01}, &ix, &iy), "world to grid in range");
    check(ix == 0 && iy == 0, "world to grid (0,0)");

    foray::nav::Vec2 center{};
    check(map.grid_to_world(0, 0, &center), "grid to world");
    check(near(center.x, 1.025) && near(center.y, 2.025), "cell center is half a cell in");

    int back_x = -1;
    int back_y = -1;
    check(map.world_to_grid(center, &back_x, &back_y), "center maps back");
    check(back_x == 0 && back_y == 0, "roundtrip stable");

    // 地图外
    check(!map.world_to_grid(foray::nav::Vec2{0.0, 2.0}, &ix, &iy), "outside map rejected");
    // 边界内侧最后一个格：x 上界 = 1.0 + 4*0.05 = 1.2
    check(map.world_to_grid(foray::nav::Vec2{1.199, 2.001}, &ix, &iy), "last column reachable");
    check(ix == 3, "last column index");
}

// ---- 2026-11-03 审计 GM-1 / GM-2 的回归测试 ----

/// GM-1：maxval 不是 255 时要归一化，否则全白图会被判成整张障碍。
void test_pgm_maxval_is_normalized() {
    for (const int maxval : {1, 15, 255}) {
        const std::string header = "P5\n2 1\n" + std::to_string(maxval) + "\n";
        std::vector<uint8_t> bytes(header.begin(), header.end());
        // 两个像素都是"最大亮度" = 全是自由
        bytes.push_back(static_cast<uint8_t>(maxval));
        bytes.push_back(static_cast<uint8_t>(maxval));
        foray::nav::GridMap map;
        std::string error;
        MapMeta meta = make_meta();
        char what[96];
        std::snprintf(what, sizeof(what), "maxval=%d loads", maxval);
        check(map.load_from_pgm_bytes(bytes, meta, &error), what);
        std::snprintf(what, sizeof(what), "maxval=%d white is free", maxval);
        check(map.cost_at(0, 0) == foray::nav::kCostFree &&
                  map.cost_at(1, 0) == foray::nav::kCostFree,
              what);
    }
}

/// GM-2：声称很大、实际数据不足的图要"返回 false"，而不是先 bad_alloc。
void test_truncated_huge_pgm_is_rejected() {
    const std::string header = "P5\n30000 30000\n255\n";
    std::vector<uint8_t> bytes(header.begin(), header.end());
    bytes.push_back(255); // 只给一个像素
    foray::nav::GridMap map;
    std::string error;
    MapMeta meta = make_meta();
    check(!map.load_from_pgm_bytes(bytes, meta, &error), "truncated huge pgm rejected");
}

} // namespace

int main() {
    test_yaml_parse();
    test_p5_and_flip();
    test_p2_ascii();
    test_unknown_and_bounds();
    test_world_grid_roundtrip();
    test_pgm_maxval_is_normalized();
    test_truncated_huge_pgm_is_rejected();

    if (g_failures == 0) {
        std::printf("ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
}
