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

// 区域表的独立测试：读得进、圈得准，且两种类型的语义不能混——
// `tunnel` 允许覆盖原图上的墙（人的显式决定），`preferred` 绝不允许。
// 这条一旦写反，车会被放进实心里，而现场看不出来。

#include <cstdio>
#include <string>
#include <vector>

#include "diesel/nav/regions.hpp"

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
    if (!condition) {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

diesel::nav::Costmap make_map() {
    diesel::nav::Costmap map;
    std::string error;
    map.init(40, 20, 0.05, diesel::nav::Vec2{0.0, 0.0}, &error);
    for (int iy = 0; iy < 20; ++iy) {
        for (int ix = 0; ix < 40; ++ix) {
            map.set_cost(ix, iy, diesel::nav::kCostFree);
        }
    }
    return map;
}

void test_contains() {
    diesel::nav::RectRegion region;
    region.force_open = false;
    region.x0 = 1.0;
    region.y0 = 2.0;
    region.x1 = 3.0;
    region.y1 = 4.0;

    check(region.contains(diesel::nav::Vec2{2.0, 3.0}), "inside");
    check(!region.contains(diesel::nav::Vec2{0.5, 3.0}), "outside left");

    region.x0 = 3.0;
    region.y0 = 4.0;
    region.x1 = 1.0;
    region.y1 = 2.0;
    check(region.contains(diesel::nav::Vec2{2.0, 3.0}), "flipped corners still work");
}

void test_load_file() {
    const char* path = "regions_test_tmp.txt";
    std::FILE* f = std::fopen(path, "w");
    if (f == nullptr) {
        check(false, "cannot write temp regions file");
        return;
    }
    std::fprintf(f, "# 隧道与走廊\n\ntunnel 3.28 7.05 3.72 7.87\npreferred 1.0 1.0 2.0 2.0\n");
    std::fclose(f);

    std::vector<diesel::nav::RectRegion> regions;
    std::string error;
    const bool ok = diesel::nav::load_regions_file(path, &regions, &error);
    std::remove(path);
    check(ok, "load ok");
    check(regions.size() == 2, "two regions");
    check(regions[0].force_open, "tunnel opens lethal cells");
    check(!regions[1].force_open, "preferred does not");
    check(regions[0].x0 == 3.28 && regions[0].y1 == 7.87, "coords parsed");
}

void test_bad_lines_are_errors() {
    const char* path = "regions_test_bad.txt";

    std::FILE* f = std::fopen(path, "w");
    std::fprintf(f, "tunnel 1.0 2.0 3.0\n");
    std::fclose(f);
    std::vector<diesel::nav::RectRegion> regions;
    std::string error;
    check(!diesel::nav::load_regions_file(path, &regions, &error), "three numbers rejected");

    f = std::fopen(path, "w");
    std::fprintf(f, "wall 1.0 2.0 3.0 4.0\n");
    std::fclose(f);
    check(!diesel::nav::load_regions_file(path, &regions, &error), "unknown kind rejected");

    std::remove(path);
}

void test_missing_file_is_error() {
    std::vector<diesel::nav::RectRegion> regions;
    std::string error;
    check(!diesel::nav::load_regions_file("__no_such_regions__.txt", &regions, &error),
          "missing file rejected");
}

/// 隧道能把墙打开；偏好型不能。
void test_apply_semantics() {
    diesel::nav::Costmap map = make_map();
    // 区域里放一段竖墙（模拟高地底下的隧道口在图上被画成黑）
    for (int ix = 18; ix < 22; ++ix) {
        for (int iy = 6; iy < 12; ++iy) {
            map.set_cost(ix, iy, diesel::nav::kCostLethal);
        }
    }
    const diesel::nav::Vec2 wall_point{18 * 0.05 + 0.025, 8 * 0.05 + 0.025};

    // 1) 偏好型：墙不动
    diesel::nav::Costmap pref_map = map;
    const std::vector<diesel::nav::RectRegion> pref_only{
        diesel::nav::RectRegion{false, 0.7, 0.2, 1.3, 0.8}};
    diesel::nav::apply_preferred_regions(pref_map, map, pref_only);
    check(pref_map.cost_at(18, 8) == diesel::nav::kCostLethal, "preferred keeps the wall");
    check(pref_map.cost_at(18, 14) == diesel::nav::kCostPreferred, "preferred marks free cells");

    // 2) 隧道型：墙被打开（人的显式决定）
    diesel::nav::Costmap tunnel_map = map;
    const std::vector<diesel::nav::RectRegion> tunnel_only{
        diesel::nav::RectRegion{true, 0.7, 0.2, 1.3, 0.8}};
    diesel::nav::apply_preferred_regions(tunnel_map, map, tunnel_only);
    check(tunnel_map.cost_at(18, 8) == diesel::nav::kCostPreferred, "tunnel opens the wall");
    check(tunnel_map.cost_at(30, 15) == diesel::nav::kCostFree, "outside untouched");
    (void)wall_point;
}

} // namespace

int main() {
    test_contains();
    test_load_file();
    test_bad_lines_are_errors();
    test_missing_file_is_error();
    test_apply_semantics();

    if (g_failures != 0) {
        std::printf("%d test(s) failed\n", g_failures);
        return 1;
    }
    std::printf("ALL TESTS PASSED\n");
    return 0;
}
