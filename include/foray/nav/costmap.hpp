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

#ifndef DIESEL__NAV__COSTMAP_HPP_
#define DIESEL__NAV__COSTMAP_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "diesel/nav/costmap_view.hpp"
#include "diesel/nav/types.hpp"

namespace foray {
namespace nav {

/// 分层代价图：把静态层、禁行层、膨胀层、动态层叠成一张**可读快照**。
///
/// 分层顺序固定，后盖前：
/// 1. 静态层：来自 `GridMap`（占据 / 自由 / 未知）；
/// 2. 禁行层：按通行档案写入的致死区（对方补给区、高地等）；
/// 3. 膨胀层：从致死格向外衰减，半径由 `min_passable_width` 反推；
/// 4. 动态层：从 `DynamicObstacle` 写入的圆，只按几何，不解释语义。
///
/// 约定：
/// - `kCostUnknown` 永远是"不可通行"，膨胀**不会**把未知刷成可走；
/// - 膨胀只降低"贴墙"的风险，不会把致死格改成可走；
/// - 所有写入操作都在构造期/装配期完成，算法只读。
class Costmap : public CostmapView {
  public:
    Costmap() = default;

    /// 按栅格尺寸初始化。分辨率必须为正。
    bool init(int width, int height, double resolution, const Vec2& origin, std::string* error);

    /// 静态层：整张拷入（要求尺寸一致）。
    bool set_static_layer(const CostmapView& static_map, std::string* error);

    /// 禁行层：把多边形内部刷成致死（测试与装配用）。
    void fill_polygon(const std::vector<Vec2>& polygon, uint8_t cost = kCostLethal);

    /// 膨胀层：从所有致死格出发做倒角距离变换，按半径给衰减代价。
    /// @param inscribed_radius 内切半径（米）：该距离内一律致死
    /// @param inflation_radius 膨胀半径（米）：超出该距离不加代价
    /// @param cost_scaling     衰减系数，越大衰减越快
    void inflate(double inscribed_radius, double inflation_radius, double cost_scaling);

    /// 动态层：按圆心与半径把格子刷成致死（不做敌我判断，见设计 §2.1）。
    void stamp_obstacles(const std::vector<DynamicObstacle>& obstacles);

    /// 动态障碍的最长存活时间（纳秒）：`stamp_ns` 早于"现在 - 这个值"的障碍按不存在处理。
    /// 0 表示不过期（不推荐：人走了、影子还在）。默认 500 ms。
    /// @note 2026-11-03 审计 CM-3：原来 `stamp_ns` 从头到尾没被读过，障碍永不过期。
    void set_obstacle_timeout_ns(int64_t timeout_ns);

    // ---- CostmapView ----
    double resolution() const override {
        return resolution_;
    }
    int width() const override {
        return width_;
    }
    int height() const override {
        return height_;
    }
    Vec2 origin() const override {
        return origin_;
    }
    int64_t stamp_ns() const override {
        return stamp_ns_;
    }
    const uint8_t* data() const override {
        return costs_.empty() ? nullptr : costs_.data();
    }
    int stride() const override {
        return width_;
    }
    uint8_t cost_at(int ix, int iy) const override;
    bool world_to_grid(const Vec2& world, int* ix, int* iy) const override;
    bool grid_to_world(int ix, int iy, Vec2* world) const override;

    /// 直接改写某一格（测试与装配用）。**会先还原膨胀前快照**——用于"层写入"。
    void set_cost(int ix, int iy, uint8_t cost);

    /// 直接覆盖某一格，**同时改快照**（不还原膨胀）。
    /// 用于"在已经膨胀好的图上挖一个洞"：起终点兜底、临时放行等。
    /// @warning 用 `set_cost()` 做这件事会把整张图的膨胀抹掉（2026-11-07 踩过）。
    void force_cost(int ix, int iy, uint8_t cost);

    /// 统计致死格数量（测试与可观测用）。
    int lethal_count() const;

  private:
    /// 回到"膨胀前"的数据：有快照就还原，没有就用当前数据建快照。
    /// 所有层写入与 `inflate()` 都先走这一步，膨胀才不会被叠两次。
    void restore_base();
    /// 把当前数据记为新的"膨胀前"快照。
    void save_base();
    double resolution_{0.05};
    Vec2 origin_{};
    int width_{0};
    int height_{0};
    std::vector<uint8_t> costs_{};
    /// 膨胀前的层数据快照：`inflate()` 每次都从这里重算，保证幂等
    /// （2026-11-03 审计 CM-1：原地膨胀会让可通行区越调越窄）。
    std::vector<uint8_t> base_costs_{};
    /// 快照是否有效。
    bool has_base_{false};
    /// 最近一次写入层的时间（纳秒），作为代价图版本。
    int64_t stamp_ns_{0};
    /// 动态障碍的最长存活时间（纳秒），0 表示不过期。
    int64_t obstacle_timeout_ns_{500000000};
};

} // namespace nav
} // namespace foray

#endif // DIESEL__NAV__COSTMAP_HPP_
