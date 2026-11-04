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

#include "foray/nav/costmap.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace foray {
namespace nav {
namespace {

int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

/// 膨胀层能写的最大代价：比"致死"低一档，避免把衰减路径当成硬障碍。
constexpr uint8_t kMaxInflatedCost = 252;

/// 点到多边形（简单多边形，不含自交）。
bool point_in_polygon(const std::vector<Vec2>& polygon, const Vec2& p) {
    if (polygon.size() < 3) {
        return false;
    }
    bool inside = false;
    const size_t n = polygon.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vec2& a = polygon[i];
        const Vec2& b = polygon[j];
        const bool crosses = ((a.y > p.y) != (b.y > p.y));
        if (!crosses) {
            continue;
        }
        const double x_at_y = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
        if (p.x < x_at_y) {
            inside = !inside;
        }
    }
    return inside;
}

} // namespace

bool Costmap::init(int width, int height, double resolution, const Vec2& origin,
                   std::string* error) {
    if (width <= 0 || height <= 0) {
        if (error) {
            *error = "costmap size must be positive";
        }
        return false;
    }
    if (resolution <= 0.0) {
        if (error) {
            *error = "costmap resolution must be positive";
        }
        return false;
    }
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    origin_ = origin;
    costs_.assign(static_cast<size_t>(width) * static_cast<size_t>(height), kCostUnknown);
    base_costs_.clear();
    has_base_ = false;
    stamp_ns_ = now_ns();
    return true;
}

bool Costmap::set_static_layer(const CostmapView& static_map, std::string* error) {
    stamp_ns_ = now_ns();
    restore_base();
    if (width_ <= 0 || height_ <= 0) {
        if (error) {
            *error = "costmap not initialized";
        }
        return false;
    }
    if (static_map.width() != width_ || static_map.height() != height_) {
        if (error) {
            *error = "static layer size mismatch";
        }
        return false;
    }
    if (std::fabs(static_map.resolution() - resolution_) > 1e-12) {
        if (error) {
            *error = "static layer resolution mismatch";
        }
        return false;
    }
    for (int iy = 0; iy < height_; ++iy) {
        for (int ix = 0; ix < width_; ++ix) {
            const uint8_t cost = static_map.cost_at(ix, iy);
            // 静态层不覆盖未知：缺数据的地方宁可保守。
            if (cost != kCostUnknown) {
                costs_[static_cast<size_t>(iy) * static_cast<size_t>(width_) +
                       static_cast<size_t>(ix)] = cost;
            }
        }
    }
    save_base();
    return true;
}

void Costmap::fill_polygon(const std::vector<Vec2>& polygon, uint8_t cost) {
    stamp_ns_ = now_ns();
    if (polygon.size() < 3) {
        return;
    }
    restore_base();
    for (int iy = 0; iy < height_; ++iy) {
        for (int ix = 0; ix < width_; ++ix) {
            Vec2 center{};
            if (!grid_to_world(ix, iy, &center)) {
                continue;
            }
            if (point_in_polygon(polygon, center)) {
                costs_[static_cast<size_t>(iy) * static_cast<size_t>(width_) +
                       static_cast<size_t>(ix)] = cost;
            }
        }
    }
    save_base();
}

void Costmap::restore_base() {
    if (has_base_ && base_costs_.size() == costs_.size()) {
        costs_ = base_costs_;
    }
}

void Costmap::save_base() {
    base_costs_ = costs_;
    has_base_ = true;
}

void Costmap::inflate(double inscribed_radius, double inflation_radius, double cost_scaling) {
    stamp_ns_ = now_ns();
    if (width_ <= 0 || height_ <= 0 || inflation_radius <= 0.0) {
        return;
    }
    // 幂等：每次都从"膨胀前"的数据重算，连调多次结果不变（审计 CM-1）。
    restore_base();
    const double inscribed = std::max(0.0, std::min(inscribed_radius, inflation_radius));
    const double decay_span = inflation_radius - inscribed;

    // 倒角距离变换（两遍），单位是米；起点为所有致死格。
    const double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> dist(costs_.size(), kInf);
    const double w_diag = std::sqrt(2.0) * resolution_;
    for (size_t i = 0; i < costs_.size(); ++i) {
        if (costs_[i] == kCostLethal) {
            dist[i] = 0.0;
        }
    }

    const int w = width_;
    const int h = height_;
    auto at = [w](int ix, int iy) {
        return static_cast<size_t>(iy) * static_cast<size_t>(w) + static_cast<size_t>(ix);
    };

    // 前向
    for (int iy = 0; iy < h; ++iy) {
        for (int ix = 0; ix < w; ++ix) {
            double& d = dist[at(ix, iy)];
            if (ix > 0) {
                d = std::min(d, dist[at(ix - 1, iy)] + resolution_);
            }
            if (iy > 0) {
                d = std::min(d, dist[at(ix, iy - 1)] + resolution_);
            }
            if (ix > 0 && iy > 0) {
                d = std::min(d, dist[at(ix - 1, iy - 1)] + w_diag);
            }
            if (ix + 1 < w && iy > 0) {
                d = std::min(d, dist[at(ix + 1, iy - 1)] + w_diag);
            }
        }
    }
    // 后向
    for (int iy = h - 1; iy >= 0; --iy) {
        for (int ix = w - 1; ix >= 0; --ix) {
            double& d = dist[at(ix, iy)];
            if (ix + 1 < w) {
                d = std::min(d, dist[at(ix + 1, iy)] + resolution_);
            }
            if (iy + 1 < h) {
                d = std::min(d, dist[at(ix, iy + 1)] + resolution_);
            }
            if (ix + 1 < w && iy + 1 < h) {
                d = std::min(d, dist[at(ix + 1, iy + 1)] + w_diag);
            }
            if (ix > 0 && iy + 1 < h) {
                d = std::min(d, dist[at(ix - 1, iy + 1)] + w_diag);
            }
        }
    }

    for (size_t i = 0; i < costs_.size(); ++i) {
        if (costs_[i] == kCostLethal || costs_[i] == kCostUnknown) {
            continue;
        }
        const double d = dist[i];
        if (!std::isfinite(d) || d >= inflation_radius) {
            continue;
        }
        if (d <= inscribed) {
            costs_[i] = kCostLethal;
            continue;
        }
        double ratio = (d - inscribed) / decay_span; // 0..1
        ratio = std::pow(ratio, std::max(0.01, cost_scaling) * 0.1);
        const double scaled = static_cast<double>(kMaxInflatedCost) * (1.0 - ratio);
        costs_[i] = static_cast<uint8_t>(std::max(1.0, std::min<double>(kMaxInflatedCost, scaled)));
    }
}

void Costmap::stamp_obstacles(const std::vector<DynamicObstacle>& obstacles) {
    stamp_ns_ = now_ns();
    if (obstacles.empty()) {
        return;
    }
    restore_base();
    for (int iy = 0; iy < height_; ++iy) {
        for (int ix = 0; ix < width_; ++ix) {
            Vec2 center{};
            if (!grid_to_world(ix, iy, &center)) {
                continue;
            }
            for (const auto& obstacle : obstacles) {
                const double dx = center.x - obstacle.center.x;
                const double dy = center.y - obstacle.center.y;
                if (std::sqrt(dx * dx + dy * dy) <= obstacle.radius) {
                    costs_[static_cast<size_t>(iy) * static_cast<size_t>(width_) +
                           static_cast<size_t>(ix)] = kCostLethal;
                    break;
                }
            }
        }
    }
    save_base();
}

uint8_t Costmap::cost_at(int ix, int iy) const {
    if (ix < 0 || iy < 0 || ix >= width_ || iy >= height_ || costs_.empty()) {
        return kCostUnknown;
    }
    return costs_[static_cast<size_t>(iy) * static_cast<size_t>(width_) + static_cast<size_t>(ix)];
}

void Costmap::set_cost(int ix, int iy, uint8_t cost) {
    if (ix < 0 || iy < 0 || ix >= width_ || iy >= height_) {
        return;
    }
    restore_base();
    costs_[static_cast<size_t>(iy) * static_cast<size_t>(width_) + static_cast<size_t>(ix)] = cost;
    save_base();
}

bool Costmap::world_to_grid(const Vec2& world, int* ix, int* iy) const {
    if (ix == nullptr || iy == nullptr || width_ <= 0 || height_ <= 0 || resolution_ <= 0.0) {
        return false;
    }
    const int cx = static_cast<int>(std::floor((world.x - origin_.x) / resolution_));
    const int cy = static_cast<int>(std::floor((world.y - origin_.y) / resolution_));
    if (cx < 0 || cy < 0 || cx >= width_ || cy >= height_) {
        return false;
    }
    *ix = cx;
    *iy = cy;
    return true;
}

bool Costmap::grid_to_world(int ix, int iy, Vec2* world) const {
    if (world == nullptr || ix < 0 || iy < 0 || ix >= width_ || iy >= height_) {
        return false;
    }
    world->x = origin_.x + (static_cast<double>(ix) + 0.5) * resolution_;
    world->y = origin_.y + (static_cast<double>(iy) + 0.5) * resolution_;
    return true;
}

int Costmap::lethal_count() const {
    int count = 0;
    for (const uint8_t cost : costs_) {
        if (cost == kCostLethal) {
            ++count;
        }
    }
    return count;
}

} // namespace nav
} // namespace foray
