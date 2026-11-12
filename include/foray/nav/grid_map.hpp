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

#ifndef FORAY__NAV__GRID_MAP_HPP_
#define FORAY__NAV__GRID_MAP_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "foray/nav/costmap_view.hpp"
#include "foray/nav/types.hpp"

namespace foray {
namespace nav {

/// 2D 栅格地图的元数据，对应 map_server 风格的 YAML（只取本仓用得到的字段）。
///
/// 字段语义与 ROS map_server 一致，便于直接吃现成的 PGM + YAML 资产：
/// - `resolution`：米/格；
/// - `origin`：[x, y, yaw]，指**图像左下角**在场地坐标系中的位置；
/// - `negate`：为 1 时像素值取反（黑白颠倒的图）；
/// - `occupied_thresh` / `free_thresh`：占用的判定阈值。
struct MapMeta {
    std::string image_path;       ///< PGM 路径（相对 YAML 所在目录或绝对路径）
    double resolution{0.05};      ///< 米/格
    double origin_x{0.0};         ///< 场地坐标系下的左下角 x（米）
    double origin_y{0.0};         ///< 场地坐标系下的左下角 y（米）
    double origin_yaw{0.0};       ///< 保留：本仓只支持 0（不旋转的地图资产）
    int negate{0};                ///< 1 = 像素取反
    double occupied_thresh{0.65}; ///< 占用阈值
    double free_thresh{0.196};    ///< 自由阈值
};

/// 2D 栅格地图：把 PGM 读成代价，并实现 `CostmapView` 的坐标换算。
///
/// 约定：
/// - 图像**第 0 行是地图的最上面一行**（PGM 的存储顺序），而栅格 `iy` 从下往上数，
///   所以读入时会做一次上下翻转——对外永远是"y 越大越靠场地上方"；
/// - 落在地图之外一律按 `kCostUnknown` 处理（保守：不可通行）；
/// - 不旋转（`origin_yaw` 必须为 0），要旋转请在上游把地图转好。
class GridMap : public CostmapView {
  public:
    GridMap() = default;

    /// 从 YAML 元数据 + PGM 文件加载。
    /// @param yaml_path 地图 YAML 的路径；`image` 字段可以是相对路径
    /// @param error     失败时写入人类可读原因（可为 nullptr）
    /// @return 成功为 true
    bool load_from_yaml(const std::string& yaml_path, std::string* error);

    /// 直接从一份 PGM 内容和元数据构造（测试用，免得落盘）。
    bool load_from_pgm_bytes(const std::vector<uint8_t>& pgm_bytes, const MapMeta& meta,
                             std::string* error);

    // ---- CostmapView ----
    double resolution() const override {
        return meta_.resolution;
    }
    int width() const override {
        return width_;
    }
    int height() const override {
        return height_;
    }
    Vec2 origin() const override {
        return Vec2{meta_.origin_x, meta_.origin_y};
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

    /// 读到的元数据（调试与测试用）。
    const MapMeta& meta() const {
        return meta_;
    }

    /// 地图是否已加载。
    bool loaded() const {
        return loaded_;
    }

  private:
    bool build_from_pixels(const std::vector<uint8_t>& pixels, int width, int height,
                           const MapMeta& meta, std::string* error);

    MapMeta meta_{};
    int width_{0};
    int height_{0};
    bool loaded_{false};
    /// 载入时刻（纳秒），作为地图资产的版本号。
    int64_t stamp_ns_{0};
    /// 按栅格下标存放的代价（已按"y 向上"翻转好）。
    std::vector<uint8_t> costs_{};
};

/// 解析 map_server 风格的 YAML 地图元数据（只认本仓用到的键，未知键忽略）。
bool parse_map_meta(const std::string& yaml_text, MapMeta* meta, std::string* error);

} // namespace nav
} // namespace foray

#endif // FORAY__NAV__GRID_MAP_HPP_
