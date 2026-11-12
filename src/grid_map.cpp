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

#include "foray/nav/grid_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
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

std::string trim(const std::string& text) {
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return std::string();
    }
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

bool parse_double(const std::string& text, double* out) {
    try {
        size_t consumed = 0;
        const double value = std::stod(text, &consumed);
        if (consumed == 0) {
            return false;
        }
        *out = value;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool read_file(const std::string& path, std::string* out, std::string* error) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        if (error) {
            *error = "cannot open file: " + path;
        }
        return false;
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    *out = buffer.str();
    return true;
}

/// 极简 PGM 解析：支持 P2（ASCII）与 P5（二进制），跳过 # 注释。
bool parse_pgm(const std::vector<uint8_t>& bytes, int* width, int* height,
               std::vector<uint8_t>* pixels, std::string* error) {
    if (bytes.empty()) {
        if (error) {
            *error = "pgm is empty";
        }
        return false;
    }
    // 头部是 ASCII，找 maxval 之后的第一个字节作为二进制起点。
    size_t pos = 0;
    auto read_token = [&bytes, &pos]() {
        while (pos < bytes.size()) {
            const uint8_t c = bytes[pos];
            if (c == '#') {
                while (pos < bytes.size() && bytes[pos] != '\n') {
                    ++pos;
                }
            } else if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
                ++pos;
            } else {
                break;
            }
        }
        const size_t begin = pos;
        while (pos < bytes.size()) {
            const uint8_t c = bytes[pos];
            if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
                break;
            }
            ++pos;
        }
        return std::string(bytes.begin() + static_cast<long>(begin),
                           bytes.begin() + static_cast<long>(pos));
    };

    const std::string magic = read_token();
    const std::string s_width = read_token();
    const std::string s_height = read_token();
    const std::string s_maxval = read_token();
    if (magic != "P5" && magic != "P2") {
        if (error) {
            *error = "unsupported pgm magic: " + magic;
        }
        return false;
    }
    int w = 0;
    int h = 0;
    int maxval = 0;
    try {
        w = std::stoi(s_width);
        h = std::stoi(s_height);
        maxval = std::stoi(s_maxval);
    } catch (const std::exception&) {
        if (error) {
            *error = "bad pgm header";
        }
        return false;
    }
    if (w <= 0 || h <= 0 || maxval <= 0 || maxval > 255) {
        if (error) {
            *error = "unsupported pgm dimensions or maxval";
        }
        return false;
    }

    const size_t count = static_cast<size_t>(w) * static_cast<size_t>(h);

    // 先校验"数据够不够"，再分配：否则一张声称 30000x30000、实际只有几十字节的图
    // 会先 bad_alloc 崩掉进程（审计 GM-2）。
    if (magic == "P2") {
        // P2 每个像素至少一个字符（含分隔符），不够就是坏的。
        if (bytes.size() - pos < count) {
            if (error) {
                *error = "pgm pixel data truncated";
            }
            return false;
        }
    } else {
        // P5：maxval 之后恰好一个空白字符，然后是 count 个原始字节。
        if (pos >= bytes.size()) {
            if (error) {
                *error = "pgm pixel data missing";
            }
            return false;
        }
        if (bytes.size() - pos - 1 < count) {
            if (error) {
                *error = "pgm pixel data truncated";
            }
            return false;
        }
        // 头部以 CRLF 结尾时，maxval 后面是两个空白字节：只跳一个会整图错位一像素（审计 GM-3）。
        while (pos < bytes.size() && (bytes[pos] == ' ' || bytes[pos] == '\t' ||
                                      bytes[pos] == '\r' || bytes[pos] == '\n')) {
            ++pos;
        }
    }

    pixels->assign(count, 0);

    if (magic == "P2") {
        for (size_t i = 0; i < count; ++i) {
            const std::string token = read_token();
            try {
                (*pixels)[i] = static_cast<uint8_t>(std::stoi(token));
            } catch (const std::exception&) {
                if (error) {
                    *error = "bad pgm pixel data";
                }
                return false;
            }
        }
    } else {
        std::copy(bytes.begin() + static_cast<long>(pos),
                  bytes.begin() + static_cast<long>(pos + count), pixels->begin());
    }

    // maxval 不一定等于 255：不归一化的话，maxval=1 的全白图会被整张判成障碍（审计 GM-1）。
    if (maxval != 255) {
        for (size_t i = 0; i < count; ++i) {
            const int scaled = static_cast<int>((*pixels)[i]) * 255 / maxval;
            (*pixels)[i] = static_cast<uint8_t>(std::min(255, std::max(0, scaled)));
        }
    }

    *width = w;
    *height = h;
    return true;
}

} // namespace

bool parse_map_meta(const std::string& yaml_text, MapMeta* meta, std::string* error) {
    if (meta == nullptr) {
        if (error) {
            *error = "meta is null";
        }
        return false;
    }
    MapMeta parsed;
    std::istringstream stream(yaml_text);
    std::string line;
    bool has_resolution = false;
    while (std::getline(stream, line)) {
        const std::string text = trim(line);
        if (text.empty() || text[0] == '#') {
            continue;
        }
        const auto colon = text.find(':');
        if (colon == std::string::npos) {
            continue;
        }
        const std::string key = trim(text.substr(0, colon));
        std::string value = trim(text.substr(colon + 1));
        // 行内注释：`image: map.pgm  # 说明`——不剥掉的话路径会带一截注释尾巴（审计 GM-4）。
        const auto hash = value.find('#');
        if (hash != std::string::npos) {
            value = trim(value.substr(0, hash));
        }
        if (!value.empty() && value.front() == '"' && value.back() == '"' && value.size() >= 2) {
            value = value.substr(1, value.size() - 2);
        }
        if (key == "image") {
            parsed.image_path = value;
        } else if (key == "resolution") {
            if (!parse_double(value, &parsed.resolution)) {
                if (error) {
                    *error = "bad resolution";
                }
                return false;
            }
            has_resolution = true;
        } else if (key == "negate") {
            // 大小写都要认：`True` 静默变 0 会悄悄翻转整张图（审计 GM-4）。
            const bool truthy = (value == "1" || value == "true" || value == "True" ||
                                 value == "TRUE" || value == "yes");
            parsed.negate = truthy ? 1 : 0;
        } else if (key == "occupied_thresh") {
            if (!parse_double(value, &parsed.occupied_thresh)) {
                if (error) {
                    *error = "bad occupied_thresh";
                }
                return false;
            }
        } else if (key == "free_thresh") {
            if (!parse_double(value, &parsed.free_thresh)) {
                if (error) {
                    *error = "bad free_thresh";
                }
                return false;
            }
        } else if (key == "origin") {
            // 形如 [x, y, yaw]
            std::string cleaned = value;
            const char* drop = "[]";
            for (const char* c = drop; *c != '\0'; ++c) {
                cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), *c), cleaned.end());
            }
            std::istringstream numbers(cleaned);
            std::string item;
            std::vector<double> values;
            while (std::getline(numbers, item, ',')) {
                double v = 0.0;
                if (parse_double(trim(item), &v)) {
                    values.push_back(v);
                }
            }
            if (values.size() >= 2) {
                parsed.origin_x = values[0];
                parsed.origin_y = values[1];
                parsed.origin_yaw = values.size() >= 3 ? values[2] : 0.0;
            }
        }
    }

    if (parsed.image_path.empty()) {
        if (error) {
            *error = "yaml has no image field";
        }
        return false;
    }
    if (!has_resolution || parsed.resolution <= 0.0) {
        if (error) {
            *error = "yaml has no valid resolution";
        }
        return false;
    }
    if (std::fabs(parsed.origin_yaw) > 1e-9) {
        if (error) {
            *error = "rotated map origin is not supported";
        }
        return false;
    }
    // 阈值反了的话，`build_from_pixels` 会先判 occupied → 整张图变成障碍且不报错
    // （审计 GM-5）。这里提前拦下来，比事后查"为什么哪儿都走不了"省事。
    if (parsed.occupied_thresh <= parsed.free_thresh) {
        if (error) {
            *error = "occupied_thresh must be greater than free_thresh";
        }
        return false;
    }
    *meta = parsed;
    return true;
}

bool GridMap::load_from_yaml(const std::string& yaml_path, std::string* error) {
    std::string yaml_text;
    if (!read_file(yaml_path, &yaml_text, error)) {
        return false;
    }
    MapMeta meta;
    if (!parse_map_meta(yaml_text, &meta, error)) {
        return false;
    }
    std::string image_path = meta.image_path;
    if (image_path.empty() || image_path.front() != '/') {
        const auto slash = yaml_path.find_last_of('/');
        const std::string dir =
            (slash == std::string::npos) ? std::string() : yaml_path.substr(0, slash + 1);
        image_path = dir + meta.image_path;
    }
    std::string raw;
    if (!read_file(image_path, &raw, error)) {
        return false;
    }
    const std::vector<uint8_t> bytes(raw.begin(), raw.end());
    return load_from_pgm_bytes(bytes, meta, error);
}

bool GridMap::load_from_pgm_bytes(const std::vector<uint8_t>& pgm_bytes, const MapMeta& meta,
                                  std::string* error) {
    int width = 0;
    int height = 0;
    std::vector<uint8_t> pixels;
    if (!parse_pgm(pgm_bytes, &width, &height, &pixels, error)) {
        return false;
    }
    return build_from_pixels(pixels, width, height, meta, error);
}

bool GridMap::build_from_pixels(const std::vector<uint8_t>& pixels, int width, int height,
                                const MapMeta& meta, std::string* error) {
    if (width <= 0 || height <= 0) {
        if (error) {
            *error = "bad map size";
        }
        return false;
    }
    if (pixels.size() < static_cast<size_t>(width) * static_cast<size_t>(height)) {
        if (error) {
            *error = "pixel count mismatch";
        }
        return false;
    }
    meta_ = meta;
    width_ = width;
    height_ = height;
    costs_.assign(static_cast<size_t>(width) * static_cast<size_t>(height), kCostUnknown);

    for (int iy = 0; iy < height; ++iy) {
        // PGM 第 0 行是地图最上面一行，栅格 iy 从下往上，所以这里翻转。
        const int src_row = height - 1 - iy;
        for (int ix = 0; ix < width; ++ix) {
            const uint8_t color = pixels[static_cast<size_t>(src_row) * static_cast<size_t>(width) +
                                         static_cast<size_t>(ix)];
            double occupancy = 0.0;
            if (meta.negate != 0) {
                occupancy = static_cast<double>(color) / 255.0;
            } else {
                occupancy = (255.0 - static_cast<double>(color)) / 255.0;
            }
            uint8_t cost = kCostUnknown;
            if (occupancy > meta.occupied_thresh) {
                cost = kCostLethal;
            } else if (occupancy < meta.free_thresh) {
                cost = kCostFree;
            }
            costs_[static_cast<size_t>(iy) * static_cast<size_t>(width) + static_cast<size_t>(ix)] =
                cost;
        }
    }

    loaded_ = true;
    // 注意：这是"载入时刻"，不是"地图版本"。契约里 stamp_ns 的用法是"与位姿/障碍时间戳
    // 比较判过期"（审计 GM-6），两者语义不同，消费者接入时别混用。
    stamp_ns_ = now_ns();
    return true;
}

uint8_t GridMap::cost_at(int ix, int iy) const {
    if (!loaded_ || ix < 0 || iy < 0 || ix >= width_ || iy >= height_) {
        return kCostUnknown;
    }
    return costs_[static_cast<size_t>(iy) * static_cast<size_t>(width_) + static_cast<size_t>(ix)];
}

bool GridMap::world_to_grid(const Vec2& world, int* ix, int* iy) const {
    if (!loaded_ || ix == nullptr || iy == nullptr) {
        return false;
    }
    const double gx = (world.x - meta_.origin_x) / meta_.resolution;
    const double gy = (world.y - meta_.origin_y) / meta_.resolution;
    const int cx = static_cast<int>(std::floor(gx));
    const int cy = static_cast<int>(std::floor(gy));
    if (cx < 0 || cy < 0 || cx >= width_ || cy >= height_) {
        return false;
    }
    *ix = cx;
    *iy = cy;
    return true;
}

bool GridMap::grid_to_world(int ix, int iy, Vec2* world) const {
    if (!loaded_ || world == nullptr || ix < 0 || iy < 0 || ix >= width_ || iy >= height_) {
        return false;
    }
    world->x = meta_.origin_x + (static_cast<double>(ix) + 0.5) * meta_.resolution;
    world->y = meta_.origin_y + (static_cast<double>(iy) + 0.5) * meta_.resolution;
    return true;
}

} // namespace nav
} // namespace foray
