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

#ifndef FORAY__NAV__PARAMS_HPP_
#define FORAY__NAV__PARAMS_HPP_

#include <string>

#include "foray/nav/astar_planner.hpp"
#include "foray/nav/nav_engine.hpp"
#include "foray/nav/pursuit_controller.hpp"

namespace foray {
namespace nav {

/// 导航参数集合：规划器 / 控制器 / 引擎三层，一次性交给调用方。
///
/// 为什么要有这一层：实车一上去参数一定要调（车体半径、限速、容差），
/// 写死在代码里就变成「改一个数、等一次 CI、上一次车」。于是把参数提到文件，
/// 两级覆盖：`shared.yaml`（本仓通用）→ `<robot>.yaml`（车种 / 赛季覆盖）。
///
/// 文件格式刻意保持**扁平**（一行一个 `key: value`，`#` 起注释），
/// 不引第三方 YAML 库——本仓的纪律是「只依赖标准库」。
/// 支持的键见 `config/shared.yaml`；**写错键名会报错**，不会静默忽略。
struct NavParams {
    AstarParams astar{};
    PursuitParams pursuit{};
    NavEngine::Options engine{};
};

/// 解析一段扁平 YAML 文本（测试与文件读取共用）。
/// @param text   文本内容；`#` 之后、行首行尾空白都忽略；空值写 `key:`。
/// @param out    读出的参数（**原地合并**：没写到的键保留原值）。
/// @param error  失败原因（未知键 / 数值解析失败 / 越界），非空指针时写入。
/// @param source 出错时用来指路的名字（文件名 / "<memory>"）。
/// @return 全部成功返回 true；任何一处失败返回 false 且 `out` 可能已部分修改——
///         调用方应先拷一份默认值再解析，失败就整份丢掉。
bool parse_params_text(const std::string& text, NavParams* out, std::string* error,
                       const char* source);

/// 从文件读参数（文件不存在 / 读不了都算失败）。
bool load_params_file(const std::string& path, NavParams* out, std::string* error);

/// 两级加载：先 `shared_path`，再用 `robot_path` 覆盖；任一文件读失败即整体失败。
bool load_params(const std::string& shared_path, const std::string& robot_path, NavParams* out,
                 std::string* error);

/// 参数合法性检查（半径为正、容差非负、限速为正……）。
/// @note 单独暴露出来，是因为调用方可能自己拼参数而不读文件。
bool validate_params(const NavParams& params, std::string* error);

} // namespace nav
} // namespace foray

#endif // FORAY__NAV__PARAMS_HPP_
