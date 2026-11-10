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

// 轨迹出图：把 nav_trace 导出的 CSV 画到地图上，输出 PPM（P6）。
//
// 为什么是 PPM：本仓只依赖标准库，不引 PNG 编码库；PPM 是「几行头 + 裸 RGB」，
// 任何看图工具 / 脚本都能转，也方便直接塞进 PR 的效果展示。
//
// 用法：
//   nav_plot --yaml maps/rmul_2027/rmul_2027.yaml --csv trace.csv --out trace.ppm
//   加 --scale 2 放大两倍（默认 1）；加 --help 看用法。
//
// 图上：浅灰=自由、深灰=致死、中灰=膨胀带、蓝色=偏好区（隧道）、
// 轨迹按速度染色（蓝=慢、红=快），绿点=起点，红点=终点。

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "foray/nav/costmap_view.hpp"
#include "foray/nav/grid_map.hpp"

namespace {

struct Rgb {
    unsigned char r{0};
    unsigned char g{0};
    unsigned char b{0};
};

void usage() {
    std::printf(
        "usage: nav_plot --yaml MAP.yaml --csv TRACE.csv [--out OUT.ppm] [--scale N]\n"
        "  MAP.yaml  地图元数据（PGM 同目录参照）\n"
        "  TRACE.csv nav_trace --csv 导出的逐帧记录\n"
        "  OUT.ppm   输出文件名，默认 trace.ppm\n"
        "  --scale   放大倍数，默认 1\n");
}

/// 代价 → 颜色。低代价（偏好）偏蓝，高代价（膨胀 → 致死）越来越深。
Rgb color_of_cost(unsigned char cost) {
    if (cost == foray::nav::kCostPreferred) {
        return Rgb{190, 220, 250};
    }
    if (cost == foray::nav::kCostFree) {
        return Rgb{245, 245, 245};
    }
    if (cost == foray::nav::kCostLethal) {
        return Rgb{35, 35, 45};
    }
    if (cost == foray::nav::kCostUnknown) {
        return Rgb{120, 120, 135};
    }
    // 膨胀带 2..252：离致死格越远越亮。252 最暗，但比致死浅，免得和墙混成一片。
    const double ratio = (static_cast<double>(cost) - 2.0) / (252.0 - 2.0);
    const double shade = 235.0 - 145.0 * ratio;
    const unsigned char gray = static_cast<unsigned char>(shade);
    return Rgb{gray, gray, static_cast<unsigned char>(gray - 10)};
}

/// 速度 → 颜色：慢=蓝，快=红。用于一眼看出「他在这儿减速了」。
Rgb color_of_speed(double speed, double max_speed) {
    if (max_speed <= 1e-9) {
        return Rgb{80, 90, 230};
    }
    double ratio = speed / max_speed;
    if (ratio < 0.0) {
        ratio = 0.0;
    }
    if (ratio > 1.0) {
        ratio = 1.0;
    }
    const unsigned char r = static_cast<unsigned char>(70.0 + 180.0 * ratio);
    const unsigned char g = static_cast<unsigned char>(90.0 - 40.0 * ratio);
    const unsigned char b = static_cast<unsigned char>(230.0 - 180.0 * ratio);
    return Rgb{r, g, b};
}

void put(std::vector<Rgb>& image, int w, int h, int x, int y, const Rgb& c, int scale) {
    for (int dy = 0; dy < scale; ++dy) {
        for (int dx = 0; dx < scale; ++dx) {
            const int px = x * scale + dx;
            const int py = y * scale + dy;
            if (px < 0 || py < 0 || px >= w || py >= h) {
                continue;
            }
            image[static_cast<size_t>(py) * static_cast<size_t>(w) + static_cast<size_t>(px)] = c;
        }
    }
}

/// Bresenham：轨迹是折线，格子和格子之间要连起来，不能只点格子。
void draw_line(std::vector<Rgb>& image, int w, int h, int x0, int y0, int x1, int y1,
               const Rgb& c, int scale) {
    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    const int sx = x0 < x1 ? 1 : -1;
    const int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        put(image, w, h, x0, y0, c, scale);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void draw_marker(std::vector<Rgb>& image, int w, int h, int cx, int cy, const Rgb& c, int scale) {
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
            put(image, w, h, cx + dx, cy + dy, c, scale);
        }
    }
}

struct Sample {
    double x{0.0};
    double y{0.0};
    double speed{0.0};
};

} // namespace

int main(int argc, char** argv) {
    std::string map_yaml;
    std::string csv_path;
    std::string out_path = "trace.ppm";
    int scale = 1;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool has = (i + 1 < argc);
        if (arg == "--help" || arg == "-h") {
            usage();
            return 0;
        } else if (arg == "--yaml" && has) {
            map_yaml = argv[++i];
        } else if (arg == "--csv" && has) {
            csv_path = argv[++i];
        } else if (arg == "--out" && has) {
            out_path = argv[++i];
        } else if (arg == "--scale" && has) {
            scale = std::atoi(argv[++i]);
        } else {
            usage();
            return 2;
        }
    }
    if (map_yaml.empty() || csv_path.empty()) {
        usage();
        return 2;
    }
    if (scale < 1) {
        scale = 1;
    }

    foray::nav::GridMap map;
    std::string error;
    if (!map.load_from_yaml(map_yaml, &error)) {
        std::printf("地图加载失败: %s\n", error.c_str());
        return 1;
    }

    std::FILE* csv = std::fopen(csv_path.c_str(), "r");
    if (csv == nullptr) {
        std::printf("读不到轨迹文件: %s\n", csv_path.c_str());
        return 1;
    }

    std::vector<Sample> samples;
    char line[512];
    (void)std::fgets(line, sizeof(line), csv); // 跳过表头
    while (std::fgets(line, sizeof(line), csv) != nullptr) {
        double t = 0.0;
        double x = 0.0;
        double y = 0.0;
        double vx = 0.0;
        double vy = 0.0;
        double clearance = 0.0;
        int status = 0;
        int replans = 0;
        const int n = std::sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%d,%d", &t, &x, &y, &vx, &vy,
                                  &clearance, &status, &replans);
        if (n < 5) {
            continue;
        }
        samples.push_back(Sample{x, y, std::hypot(vx, vy)});
    }
    std::fclose(csv);

    if (samples.empty()) {
        std::printf("轨迹文件里没有数据点: %s\n", csv_path.c_str());
        return 1;
    }

    const int cell_w = map.width();
    const int cell_h = map.height();
    const int img_w = cell_w * scale;
    const int img_h = cell_h * scale;
    const double res = map.resolution();
    const foray::nav::Vec2 origin = map.origin();

    std::vector<Rgb> image(static_cast<size_t>(img_w) * static_cast<size_t>(img_h));
    for (int y = 0; y < cell_h; ++y) {
        for (int x = 0; x < cell_w; ++x) {
            // 图像行是「从上往下」，地图 y 是「往上」，所以翻转一次。
            put(image, img_w, img_h, x, cell_h - 1 - y, color_of_cost(map.cost_at(x, y)), scale);
        }
    }

    double max_speed = 0.0;
    for (const Sample& s : samples) {
        max_speed = std::max(max_speed, s.speed);
    }

    auto to_cell = [&](double wx, double wy, int* cx, int* cy) {
        *cx = static_cast<int>(std::floor((wx - origin.x) / res));
        *cy = static_cast<int>(std::floor((wy - origin.y) / res));
    };

    for (size_t i = 1; i < samples.size(); ++i) {
        int x0 = 0;
        int y0 = 0;
        int x1 = 0;
        int y1 = 0;
        to_cell(samples[i - 1].x, samples[i - 1].y, &x0, &y0);
        to_cell(samples[i].x, samples[i].y, &x1, &y1);
        const Rgb color = color_of_speed(samples[i].speed, max_speed);
        draw_line(image, img_w, img_h, x0, cell_h - 1 - y0, x1, cell_h - 1 - y1, color, scale);
    }

    int sx = 0;
    int sy = 0;
    int gx = 0;
    int gy = 0;
    to_cell(samples.front().x, samples.front().y, &sx, &sy);
    to_cell(samples.back().x, samples.back().y, &gx, &gy);
    draw_marker(image, img_w, img_h, sx, cell_h - 1 - sy, Rgb{0, 190, 60}, scale);
    draw_marker(image, img_w, img_h, gx, cell_h - 1 - gy, Rgb{230, 40, 60}, scale);

    std::FILE* out = std::fopen(out_path.c_str(), "wb");
    if (out == nullptr) {
        std::printf("写不出图: %s\n", out_path.c_str());
        return 1;
    }
    std::fprintf(out, "P6\n%d %d\n255\n", img_w, img_h);
    std::fwrite(image.data(), sizeof(Rgb), image.size(), out);
    std::fclose(out);

    std::printf("已出图: %s（%d x %d，%zu 帧，最快 %.2f m/s）\n", out_path.c_str(), img_w, img_h,
                samples.size(), max_speed);
    return 0;
}
