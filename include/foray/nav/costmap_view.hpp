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

#ifndef DIESEL__NAV__COSTMAP_VIEW_HPP_
#define DIESEL__NAV__COSTMAP_VIEW_HPP_

#include <cstdint>

#include "diesel/nav/types.hpp"

namespace foray {
namespace nav {

/// 代价编码：**数值越小越划算**，与语义顺序一致（按评审意见统一）。
///
/// - `kCostPreferred = 0`：偏好区（例如隧道），比自由更便宜；
/// - `kCostFree = 1`：普通可行区域；
/// - `2..252`：膨胀层衰减带，越大越"贵"；
/// - `kCostUnknown = 253` / `kCostLethal = 255`：不可通行，两者分开编码只为日志与测试好区分；
/// - `kCostReserved = 254`：**保留、不许写入**。它夹在未知与致死之间，任何误写都会被
///   当成"可通行"（审计 CM-9）；`passable()` 把它按不可通行处理，留个保险。
///
/// @warning 规划器必须把代价当作**边权重**使用，不得二值化成"能走/不能走"，
///          否则代价场失效（详见 `planner.hpp` 契约）。
enum CostValue : uint8_t {
    kCostPreferred = 0,
    kCostFree = 1,
    /// 未知空间（地图未标、或地图外）：保守视为不可通行。
    kCostUnknown = 253,
    /// 保留值：不许写入；`passable()` 按不可通行处理（审计 CM-9）。
    kCostReserved = 254,
    /// 致死：禁行区、高地、动态障碍占位。
    kCostLethal = 255,
};

/// 代价图的**只读快照**。算法只读它，永不就地修改。
///
/// 由代价图构建器（静态层 + 禁行层 + 膨胀层 + 动态层）生成；
/// 规划与控制都拿同一份快照，避免"规划时用旧图、控制时用新图"。
class CostmapView {
  public:
    virtual ~CostmapView() = default;

    /// 分辨率（米/格）。2027 赛季默认 0.05。
    virtual double resolution() const = 0;

    /// 栅格宽度（格数）。
    virtual int width() const = 0;

    /// 栅格高度（格数）。
    virtual int height() const = 0;

    /// 栅格原点（场地坐标系，米）。指**第 (0,0) 格的左下角**。
    virtual Vec2 origin() const = 0;

    /// 快照的时间戳（纳秒），用作**地图/代价图版本**。
    ///
    /// 用途：与位姿、动态障碍的时间戳比较——比它旧的数据视为过期。
    /// 返回 0 表示"实现不提供版本信息"，调用方应按"无法判断"处理。
    virtual int64_t stamp_ns() const = 0;

    /// 取代价；越界返回 `kCostUnknown`。
    virtual uint8_t cost_at(int ix, int iy) const = 0;

    /// 世界坐标 → 栅格下标；越界返回 false。
    virtual bool world_to_grid(const Vec2& world, int* ix, int* iy) const = 0;

    /// 栅格下标 → 世界坐标（该格中心）；越界返回 false。
    virtual bool grid_to_world(int ix, int iy, Vec2* world) const = 0;

    // ---- 热路径的批量只读入口（规划器用；逐格走虚函数太慢）----

    /// 连续代价数组的首地址，按行存储、每行 `stride()` 个字节。
    ///
    /// 返回 `nullptr` 表示**本实现不暴露连续内存**，调用方必须退回 `cost_at()`。
    /// 只读：调用方不得写入，也不得假设它的生命周期长于本对象。
    virtual const uint8_t* data() const {
        return nullptr;
    }

    /// 每行的字节数（通常等于 `width()`）。`data()` 为 nullptr 时无意义。
    virtual int stride() const {
        return width();
    }

    /// 该格是否可通行（非致死、非未知、非保留值）。
    bool passable(int ix, int iy) const {
        const uint8_t c = cost_at(ix, iy);
        return c != kCostLethal && c != kCostUnknown && c != kCostReserved;
    }
};

} // namespace nav
} // namespace foray

#endif // DIESEL__NAV__COSTMAP_VIEW_HPP_
