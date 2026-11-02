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

// types 的独立测试：兵种接口预留是否真的"只换数值、不改接口"。

#include <cstdio>
#include <cstring>

#include "foray/nav/types.hpp"

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
    if (!condition) {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

void test_default_is_sentinel() {
    foray::nav::Capability cap;
    check(cap.kind == foray::nav::UnitKind::kSentinel, "default kind is sentinel");
    check(!cap.can_climb_highland, "sentinel cannot climb highland by default");
}

void test_kind_strings() {
    check(std::strcmp(foray::nav::to_string(foray::nav::UnitKind::kSentinel), "sentinel") == 0,
          "sentinel string");
    check(std::strcmp(foray::nav::to_string(foray::nav::UnitKind::kInfantry), "infantry") == 0,
          "infantry string");
    check(std::strcmp(foray::nav::to_string(foray::nav::UnitKind::kHeavy), "heavy") == 0,
          "heavy string");
}

/// 同一套接口换数值就能表达另一个兵种——这是本次预留要证明的事。
void test_other_kinds_fit_same_interface() {
    foray::nav::TraversalProfile infantry;
    infantry.capability.kind = foray::nav::UnitKind::kInfantry;
    infantry.capability.min_passable_width = 0.60;
    infantry.capability.max_slope_deg = 15.0;
    infantry.capability.can_climb_highland = true;

    foray::nav::TraversalProfile heavy;
    heavy.capability.kind = foray::nav::UnitKind::kHeavy;
    heavy.capability.min_passable_width = 0.80;
    heavy.capability.max_slope_deg = 10.0;
    heavy.capability.can_climb_highland = true;

    check(infantry.capability.min_passable_width < heavy.capability.min_passable_width,
          "infantry passes narrower gaps than heavy");
    check(infantry.capability.kind != heavy.capability.kind, "kinds stay distinguishable");
}

} // namespace

int main() {
    test_default_is_sentinel();
    test_kind_strings();
    test_other_kinds_fit_same_interface();

    if (g_failures != 0) {
        std::printf("%d test(s) failed\n", g_failures);
        return 1;
    }
    std::printf("ALL TESTS PASSED\n");
    return 0;
}
