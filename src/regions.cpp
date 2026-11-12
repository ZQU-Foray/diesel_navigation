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

// 区域表：把「栅格图看不出来的通道」补回代价图。
//
// 2027 场地的隧道在高地底下，而 PGM 把高地画成实心黑块——图上看着就是墙。
// 按车体半径（0.32 m）膨胀之后更糟：整条隧道连同两侧都被算成致死，A* 只能绕整个高地。
// 所以需要一层「人来标」的信息，而且是**能覆盖原图致死格**的那种。

#include "diesel/nav/regions.hpp"

#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>

namespace diesel {
namespace nav {

bool RectRegion::contains(const Vec2& point) const {
    const double lo_x = x0 < x1 ? x0 : x1;
    const double hi_x = x0 < x1 ? x1 : x0;
    const double lo_y = y0 < y1 ? y0 : y1;
    const double hi_y = y0 < y1 ? y1 : y0;
    return point.x >= lo_x && point.x <= hi_x && point.y >= lo_y && point.y <= hi_y;
}

bool load_regions_file(const std::string& path, std::vector<RectRegion>* out, std::string* error) {
    if (out == nullptr) {
        if (error != nullptr) {
            *error = "load_regions_file: 输出为空";
        }
        return false;
    }
    std::ifstream file(path);
    if (!file) {
        if (error != nullptr) {
            *error = "读不到区域表：" + path;
        }
        return false;
    }

    out->clear();
    std::string line;
    int line_number = 0;
    while (std::getline(file, line)) {
        ++line_number;
        const std::string::size_type hash = line.find('#');
        if (hash != std::string::npos) {
            line.erase(hash);
        }
        std::istringstream stream(line);
        std::string kind;
        if (!(stream >> kind)) {
            continue; // 空行
        }
        double values[4] = {0.0, 0.0, 0.0, 0.0};
        std::string trailing;
        const bool shape_ok =
            static_cast<bool>(stream >> values[0] >> values[1] >> values[2] >> values[3]) &&
            !(stream >> trailing);
        if (!shape_ok) {
            if (error != nullptr) {
                *error = path + ":" + std::to_string(line_number) +
                         ": 格式应为「类型 x0 y0 x1 y1」：\"" + line + "\"";
            }
            return false;
        }
        bool force_open = false;
        if (kind == "tunnel") {
            force_open = true;
        } else if (kind != "preferred") {
            if (error != nullptr) {
                *error = path + ":" + std::to_string(line_number) +
                         ": 类型只能是 tunnel 或 preferred：\"" + kind + "\"";
            }
            return false;
        }
        if (!std::isfinite(values[0]) || !std::isfinite(values[1]) || !std::isfinite(values[2]) ||
            !std::isfinite(values[3])) {
            if (error != nullptr) {
                *error = path + ":" + std::to_string(line_number) + ": 有非法数值";
            }
            return false;
        }
        out->push_back(RectRegion{force_open, values[0], values[1], values[2], values[3]});
    }
    if (out->empty()) {
        if (error != nullptr) {
            *error = "区域表是空的：" + path;
        }
        return false;
    }
    return true;
}

void apply_preferred_regions(Costmap& map, const CostmapView& source,
                             const std::vector<RectRegion>& regions) {
    if (regions.empty()) {
        return;
    }
    for (int iy = 0; iy < map.height(); ++iy) {
        for (int ix = 0; ix < map.width(); ++ix) {
            Vec2 center{};
            if (!map.grid_to_world(ix, iy, &center)) {
                continue;
            }
            const RectRegion* hit = nullptr;
            for (const RectRegion& region : regions) {
                if (region.contains(center)) {
                    hit = &region;
                    break;
                }
            }
            if (hit == nullptr) {
                continue;
            }
            // 未知格永远不打开：没有测绘数据的地方，不许靠区域表猜。
            const uint8_t original = source.cost_at(ix, iy);
            if (original == kCostUnknown) {
                continue;
            }
            // 偏好型只改本来就能走的地方；隧道型允许覆盖致死格（人的显式决定）。
            if (!hit->force_open && (original == kCostLethal || original == kCostUnknown)) {
                continue;
            }
            map.force_cost(ix, iy, kCostPreferred);
        }
    }
}

} // namespace nav
} // namespace diesel
