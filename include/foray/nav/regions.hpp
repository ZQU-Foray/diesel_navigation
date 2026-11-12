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

#ifndef FORAY__NAV__REGIONS_HPP_
#define FORAY__NAV__REGIONS_HPP_

#include <string>
#include <vector>

#include "foray/nav/costmap.hpp"
#include "foray/nav/types.hpp"

namespace foray {
namespace nav {

/// 地图上的一块矩形区域（世界坐标，米）。
///
/// 用来表达「地图栅格里看不出来的东西」。2027 场地上有两类：
/// - **隧道**：高地底下真的能穿过去，但 PGM 把高地画成实心黑块——图上看着是墙。
///   这类区域必须**强行打开**（`force_open = true`），否则规划器永远绕高地；
/// - **偏好**：本来就是空地，只是队伍想让它优先走（例如指定走廊）。
struct RectRegion {
    /// 是否允许把「原图上的致死格」打开。隧道 = true，偏好 = false。
    bool force_open{false};
    double x0{0.0};
    double y0{0.0};
    double x1{0.0};
    double y1{0.0};

    /// 点是否落在矩形内（含边界，按两对角点自动归一化）。
    bool contains(const Vec2& point) const;
};

/// 读区域表文件。格式：一行一块，`类型 x0 y0 x1 y1`（米），`#` 起注释，空行忽略。
/// 类型：`tunnel`（隧道，可打开致死格）或 `preferred`（本来就能走，只是更想走）。
/// 读不通或数字不合法时返回 false 并写 `error`（带行号）。
bool load_regions_file(const std::string& path, std::vector<RectRegion>* out, std::string* error);

/// 把区域内的格子刷成**偏好**（`kCostPreferred`）。
/// - `force_open == true`（隧道）：连原图上的致死格也打开——这是人的显式覆盖；
/// - `force_open == false`（偏好）：只改本来就可通行的格子；
/// - **未知格永远不打开**（没有测绘数据的地方，不许靠区域表猜）。
/// @note 必须在膨胀**之后**调用，否则膨胀会把隧道口重新糊上。
void apply_preferred_regions(Costmap& map, const CostmapView& source,
                             const std::vector<RectRegion>& regions);

} // namespace nav
} // namespace foray

#endif // FORAY__NAV__REGIONS_HPP_
