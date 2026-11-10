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

// 扁平 YAML 参数读取：只为「把参数从代码里挪出来」这件事，不追求通用 YAML。
//
// 语法就三条：
//   1. 一行一个 `key: value`；行首行尾空白忽略；
//   2. `#` 之后是注释（不区分它是否在引号里——值里不需要 `#`）；
//   3. 空行忽略。
//
// 为什么不引第三方库：本仓（见 .foray-layer / CMakeLists）的纪律是只依赖标准库，
// 而这里要的全部能力就是一个 map。

#include "foray/nav/params.hpp"

#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

namespace foray {
namespace nav {
namespace {

/// 解析现场：哪个文件、第几行——报错要能指到地方，不然调参调到天亮。
struct Context {
    const char* source;
    int line;
    std::string* error;
};

std::string trim(const std::string& in) {
    const char* kWhite = " \t\r\n";
    const std::string::size_type first = in.find_first_not_of(kWhite);
    if (first == std::string::npos) {
        return std::string();
    }
    const std::string::size_type last = in.find_last_not_of(kWhite);
    return in.substr(first, last - first + 1);
}

bool fail(const Context& ctx, const std::string& message) {
    if (ctx.error != nullptr) {
        *ctx.error = std::string(ctx.source != nullptr ? ctx.source : "<params>") + ":" +
                     std::to_string(ctx.line) + ": " + message;
    }
    return false;
}

bool parse_double(const std::string& text, double* out) {
    if (text.empty()) {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (end == text.c_str() || errno == ERANGE || *end != '\0') {
        return false;
    }
    *out = value;
    return true;
}

bool parse_int(const std::string& text, int* out) {
    if (text.empty()) {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const long value = std::strtol(text.c_str(), &end, 10);
    if (end == text.c_str() || errno == ERANGE || *end != '\0') {
        return false;
    }
    *out = static_cast<int>(value);
    return true;
}

bool parse_bool(const std::string& text, bool* out) {
    if (text == "true" || text == "1" || text == "yes" || text == "on") {
        *out = true;
        return true;
    }
    if (text == "false" || text == "0" || text == "no" || text == "off") {
        *out = false;
        return true;
    }
    return false;
}

template <typename T>
bool read_number(const std::string& key, const std::string& text, T* field,
                 bool (*parser)(const std::string&, T*), const Context& ctx) {
    if (!parser(text, field)) {
        return fail(ctx, "参数 " + key + " 的值不合法：\"" + text + "\"");
    }
    return true;
}

/// 把一个键写到参数结构里；键不认得就报错（拼错不许静默忽略——静默的下场是
/// 「调了半天发现改的是另一个名字」）。
bool apply_key(const std::string& key, const std::string& value, NavParams& p, const Context& ctx) {
    if (key == "engine.robot_radius") {
        return read_number(key, value, &p.engine.robot_radius, parse_double, ctx);
    }
    if (key == "engine.deviation_replan_dist") {
        return read_number(key, value, &p.engine.deviation_replan_dist, parse_double, ctx);
    }
    if (key == "engine.max_replans") {
        return read_number(key, value, &p.engine.max_replans, parse_int, ctx);
    }
    if (key == "engine.stall_window_s") {
        return read_number(key, value, &p.engine.stall_window_s, parse_double, ctx);
    }
    if (key == "engine.stall_progress_m") {
        return read_number(key, value, &p.engine.stall_progress_m, parse_double, ctx);
    }
    if (key == "engine.preview_time_s") {
        return read_number(key, value, &p.engine.preview_time_s, parse_double, ctx);
    }
    if (key == "engine.creep_scale") {
        return read_number(key, value, &p.engine.creep_scale, parse_double, ctx);
    }
    if (key == "engine.heading_tolerance") {
        return read_number(key, value, &p.engine.heading_tolerance, parse_double, ctx);
    }

    if (key == "astar.cost_weight") {
        return read_number(key, value, &p.astar.cost_weight, parse_double, ctx);
    }
    if (key == "astar.prefer_weight") {
        return read_number(key, value, &p.astar.prefer_weight, parse_double, ctx);
    }
    if (key == "astar.max_expansions") {
        return read_number(key, value, &p.astar.max_expansions, parse_int, ctx);
    }
    if (key == "astar.smooth") {
        return read_number(key, value, &p.astar.smooth, parse_bool, ctx);
    }

    if (key == "pursuit.lookahead_min") {
        return read_number(key, value, &p.pursuit.lookahead_min, parse_double, ctx);
    }
    if (key == "pursuit.lookahead_max") {
        return read_number(key, value, &p.pursuit.lookahead_max, parse_double, ctx);
    }
    if (key == "pursuit.lookahead_time") {
        return read_number(key, value, &p.pursuit.lookahead_time, parse_double, ctx);
    }
    if (key == "pursuit.max_linear") {
        return read_number(key, value, &p.pursuit.max_linear, parse_double, ctx);
    }
    if (key == "pursuit.max_accel") {
        return read_number(key, value, &p.pursuit.max_accel, parse_double, ctx);
    }
    if (key == "pursuit.approach_scaling_dist") {
        return read_number(key, value, &p.pursuit.approach_scaling_dist, parse_double, ctx);
    }
    if (key == "pursuit.min_approach_speed") {
        return read_number(key, value, &p.pursuit.min_approach_speed, parse_double, ctx);
    }
    if (key == "pursuit.curvature_min") {
        return read_number(key, value, &p.pursuit.curvature_min, parse_double, ctx);
    }
    if (key == "pursuit.curvature_max") {
        return read_number(key, value, &p.pursuit.curvature_max, parse_double, ctx);
    }
    if (key == "pursuit.reduction_ratio_at_high_curvature") {
        return read_number(key, value, &p.pursuit.reduction_ratio_at_high_curvature, parse_double,
                           ctx);
    }
    if (key == "pursuit.curvature_forward_dist") {
        return read_number(key, value, &p.pursuit.curvature_forward_dist, parse_double, ctx);
    }
    if (key == "pursuit.stop_tolerance") {
        return read_number(key, value, &p.pursuit.stop_tolerance, parse_double, ctx);
    }
    if (key == "pursuit.align_heading_at_goal") {
        return read_number(key, value, &p.pursuit.align_heading_at_goal, parse_bool, ctx);
    }
    if (key == "pursuit.heading_gain") {
        return read_number(key, value, &p.pursuit.heading_gain, parse_double, ctx);
    }
    if (key == "pursuit.max_omega") {
        return read_number(key, value, &p.pursuit.max_omega, parse_double, ctx);
    }
    if (key == "pursuit.heading_tolerance") {
        return read_number(key, value, &p.pursuit.heading_tolerance, parse_double, ctx);
    }

    return fail(ctx, "不认识的参数：" + key + "（键名见 config/shared.yaml）");
}

} // namespace

bool validate_params(const NavParams& params, std::string* error) {
    auto bad = [error](const std::string& message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };

    if (!(params.engine.robot_radius > 0.0)) {
        return bad("engine.robot_radius 必须为正");
    }
    if (!(params.engine.deviation_replan_dist > 0.0)) {
        return bad("engine.deviation_replan_dist 必须为正");
    }
    if (params.engine.max_replans < 0) {
        return bad("engine.max_replans 不能为负");
    }
    if (!(params.engine.stall_window_s > 0.0)) {
        return bad("engine.stall_window_s 必须为正");
    }
    if (params.engine.stall_progress_m < 0.0) {
        return bad("engine.stall_progress_m 不能为负");
    }
    if (params.engine.preview_time_s < 0.0) {
        return bad("engine.preview_time_s 不能为负");
    }
    if (!(params.engine.creep_scale > 0.0 && params.engine.creep_scale <= 1.0)) {
        return bad("engine.creep_scale 必须落在 (0, 1]");
    }
    if (!(params.engine.heading_tolerance > 0.0)) {
        return bad("engine.heading_tolerance 必须为正");
    }

    if (!(params.astar.cost_weight > 0.0)) {
        return bad("astar.cost_weight 必须为正");
    }
    if (!(params.astar.prefer_weight > 0.0 && params.astar.prefer_weight <= 1.0)) {
        return bad("astar.prefer_weight 必须落在 (0, 1]——大于 1 就变成「不想走这里」了");
    }
    if (params.astar.max_expansions <= 0) {
        return bad("astar.max_expansions 必须为正");
    }

    if (!(params.pursuit.lookahead_min > 0.0)) {
        return bad("pursuit.lookahead_min 必须为正");
    }
    if (params.pursuit.lookahead_max < params.pursuit.lookahead_min) {
        return bad("pursuit.lookahead_max 不能小于 lookahead_min");
    }
    if (!(params.pursuit.lookahead_time > 0.0)) {
        return bad("pursuit.lookahead_time 必须为正");
    }
    if (!(params.pursuit.max_linear > 0.0)) {
        return bad("pursuit.max_linear 必须为正");
    }
    if (!(params.pursuit.max_accel > 0.0)) {
        return bad("pursuit.max_accel 必须为正");
    }
    if (params.pursuit.approach_scaling_dist < 0.0) {
        return bad("pursuit.approach_scaling_dist 不能为负");
    }
    if (params.pursuit.min_approach_speed < 0.0) {
        return bad("pursuit.min_approach_speed 不能为负");
    }
    if (!(params.pursuit.curvature_max > params.pursuit.curvature_min)) {
        return bad("pursuit.curvature_max 必须大于 curvature_min");
    }
    if (!(params.pursuit.reduction_ratio_at_high_curvature > 0.0 &&
          params.pursuit.reduction_ratio_at_high_curvature <= 1.0)) {
        return bad("pursuit.reduction_ratio_at_high_curvature 必须落在 (0, 1]");
    }
    if (!(params.pursuit.stop_tolerance > 0.0)) {
        return bad("pursuit.stop_tolerance 必须为正");
    }
    if (params.pursuit.heading_gain < 0.0) {
        return bad("pursuit.heading_gain 不能为负");
    }
    if (params.pursuit.max_omega < 0.0) {
        return bad("pursuit.max_omega 不能为负");
    }
    if (!(params.pursuit.heading_tolerance > 0.0)) {
        return bad("pursuit.heading_tolerance 必须为正");
    }
    return true;
}

bool parse_params_text(const std::string& text, NavParams* out, std::string* error,
                       const char* source) {
    if (out == nullptr) {
        if (error != nullptr) {
            *error = "parse_params_text: 输出参数为空";
        }
        return false;
    }

    std::istringstream stream(text);
    std::string raw;
    int line = 0;
    while (std::getline(stream, raw)) {
        ++line;
        const std::string::size_type hash = raw.find('#');
        if (hash != std::string::npos) {
            raw.erase(hash);
        }
        const std::string entry = trim(raw);
        if (entry.empty()) {
            continue;
        }
        const std::string::size_type colon = entry.find(':');
        const Context ctx{source, line, error};
        if (colon == std::string::npos) {
            return fail(ctx, "缺少冒号（格式：key: value）：\"" + entry + "\"");
        }
        const std::string key = trim(entry.substr(0, colon));
        const std::string value = trim(entry.substr(colon + 1));
        if (key.empty()) {
            return fail(ctx, "键名为空");
        }
        if (!apply_key(key, value, *out, ctx)) {
            return false;
        }
    }
    return validate_params(*out, error);
}

bool load_params_file(const std::string& path, NavParams* out, std::string* error) {
    std::ifstream file(path);
    if (!file) {
        if (error != nullptr) {
            *error = "读不到参数文件：" + path;
        }
        return false;
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return parse_params_text(buffer.str(), out, error, path.c_str());
}

bool load_params(const std::string& shared_path, const std::string& robot_path, NavParams* out,
                 std::string* error) {
    if (out == nullptr) {
        if (error != nullptr) {
            *error = "load_params: 输出参数为空";
        }
        return false;
    }
    NavParams merged{};
    if (!load_params_file(shared_path, &merged, error)) {
        return false;
    }
    if (!robot_path.empty()) {
        if (!load_params_file(robot_path, &merged, error)) {
            return false;
        }
    }
    *out = merged;
    return true;
}

} // namespace nav
} // namespace foray
