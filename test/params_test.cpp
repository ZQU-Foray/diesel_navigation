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

// 参数文件的独立测试：能不能读、键名写错会不会报、两级覆盖对不对、越界拦不拦。
//
// 参数读错的代价特别高——它不会崩，只会「车走得怪」，然后人去找算法的问题。
// 所以这里专门测「错了要说话」的那几条。

#include <cstdio>
#include <string>

#include "foray/nav/params.hpp"

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
    if (!condition) {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

void test_defaults_match_struct_defaults() {
    foray::nav::NavParams params;

    check(params.engine.robot_radius == 0.32, "engine.robot_radius default 0.32 (593 mm 车体)");
    check(params.engine.max_replans == 3, "engine.max_replans default 3");
    check(params.astar.cost_weight == 3.0, "astar.cost_weight default 3.0");
    check(params.pursuit.max_linear == 2.5, "pursuit.max_linear default 2.5");
    check(params.pursuit.align_heading_at_goal == false, "align heading default off");
}

void test_parse_overrides_numbers_and_bools() {
    foray::nav::NavParams params;
    std::string error;
    const char* text = "# 注释行\n"
                       "\n"
                       "engine.robot_radius: 0.31\n"
                       "engine.max_replans: 5    # 行尾注释\n"
                       "astar.smooth: false\n"
                       "pursuit.max_linear: 1.75\n";

    check(foray::nav::parse_params_text(text, &params, &error, "<test>"), "parse ok");
    check(error.empty(), "no error text");
    check(params.engine.robot_radius == 0.31, "robot_radius overridden");
    check(params.engine.max_replans == 5, "max_replans overridden");
    check(params.astar.smooth == false, "smooth overridden");
    check(params.pursuit.max_linear == 1.75, "max_linear overridden");
    // 没写到的键保持默认值
    check(params.engine.deviation_replan_dist == 0.50, "untouched key keeps default");
}

/// 两级：先 shared，再 robot——robot 只覆盖它写到的键。
void test_two_level_override() {
    foray::nav::NavParams params;
    std::string error;

    const char* shared = "engine.robot_radius: 0.25\npursuit.max_linear: 2.5\n";
    const char* robot = "engine.robot_radius: 0.40\n";

    check(foray::nav::parse_params_text(shared, &params, &error, "<shared>"), "shared parses");
    check(foray::nav::parse_params_text(robot, &params, &error, "<robot>"), "robot parses");
    check(params.engine.robot_radius == 0.40, "robot overrides shared");
    check(params.pursuit.max_linear == 2.5, "shared value survives robot layer");
}

void test_unknown_key_is_error() {
    foray::nav::NavParams params;
    std::string error;
    check(!foray::nav::parse_params_text("engine.robot_radiuz: 0.3\n", &params, &error, "x.yaml"),
          "typo rejected");
    check(contains(error, "engine.robot_radiuz"), "error names the bad key");
    check(contains(error, "x.yaml"), "error names the file");
}

void test_bad_number_is_error() {
    foray::nav::NavParams params;
    std::string error;
    check(!foray::nav::parse_params_text("engine.robot_radius: abc\n", &params, &error, "x.yaml"),
          "non-numeric rejected");
    check(contains(error, "engine.robot_radius"), "error names the key");
}

void test_missing_colon_is_error() {
    foray::nav::NavParams params;
    std::string error;
    check(!foray::nav::parse_params_text("engine.robot_radius 0.3\n", &params, &error, "x.yaml"),
          "missing colon rejected");
    check(contains(error, ":1"), "error points at line 1");
}

void test_range_validation() {
    foray::nav::NavParams params;
    std::string error;

    check(!foray::nav::parse_params_text("engine.robot_radius: -0.2\n", &params, &error, "x.yaml"),
          "negative radius rejected");

    foray::nav::NavParams params2;
    check(!foray::nav::parse_params_text("engine.creep_scale: 1.5\n", &params2, &error, "x.yaml"),
          "creep_scale above 1 rejected");

    foray::nav::NavParams params3;
    check(
        !foray::nav::parse_params_text("pursuit.curvature_max: 0.2\n", &params3, &error, "x.yaml"),
        "curvature_max below curvature_min rejected");
}

void test_missing_file_is_error() {
    foray::nav::NavParams params;
    std::string error;
    check(!foray::nav::load_params_file("config/__no_such_file__.yaml", &params, &error),
          "missing file rejected");
    check(contains(error, "__no_such_file__"), "error names the missing file");
}

/// 仓库里真实的两份配置必须能读通——否则「文档里写的参数」和「能跑的参数」会悄悄分叉。
void test_repo_config_files_load() {
    foray::nav::NavParams params;
    std::string error;
    const bool ok =
        foray::nav::load_params("config/shared.yaml", "config/rmul_2027.yaml", &params, &error);
    if (!ok) {
        std::printf("NOTE: 仓库配置没找到（在构建目录里跑测试时正常）：%s\n", error.c_str());
        return; // 构建目录不是仓库根时不算失败
    }
    check(params.engine.robot_radius > 0.0, "repo config gives a usable radius");
    check(params.pursuit.max_linear > 0.0, "repo config gives a usable speed limit");
}

} // namespace

int main() {
    test_defaults_match_struct_defaults();
    test_parse_overrides_numbers_and_bools();
    test_two_level_override();
    test_unknown_key_is_error();
    test_bad_number_is_error();
    test_missing_colon_is_error();
    test_range_validation();
    test_missing_file_is_error();
    test_repo_config_files_load();

    if (g_failures != 0) {
        std::printf("%d test(s) failed\n", g_failures);
        return 1;
    }
    std::printf("ALL TESTS PASSED\n");
    return 0;
}
