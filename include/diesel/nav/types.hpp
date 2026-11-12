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

#ifndef DIESEL__NAV__TYPES_HPP_
#define DIESEL__NAV__TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace diesel {
namespace nav {

/// 场地平面上的点，单位米，坐标系为**场地坐标系**（原点对齐规则定义的原点，不是开机位置）。
struct Vec2 {
    double x{0.0};
    double y{0.0};
};

/// 位姿：位置 + 朝向（弧度，逆时针为正）。
struct Pose2 {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

/// 阵营。只影响"对方补给区禁行"这一条，不影响地图本身。
enum class Side {
    kRed = 0,
    kBlue = 1,
};

/// 兵种。本仓**只消费**哨兵（`kSentinel`）；其余两项是接口预留：
/// 队内已计划让半自动步兵与重装复用本仓的规划器，届时只换 `Capability` 的数值，
/// 不改接口（见 decision.md「2026-11-02 兵种接口预留」）。
enum class UnitKind {
    kSentinel = 0,
    kInfantry,
    kHeavy,
};

/// 兵种能力。只描述"这台车能不能"，不描述"想不想"。
/// @note 2027 赛季本仓只服务哨兵；`kind` 与其余字段保留，供将来扩展。
struct Capability {
    /// 兵种。默认哨兵；步兵/重装目前只用于接口预留。
    UnitKind kind{UnitKind::kSentinel};
    /// 最大爬坡角（度）。
    double max_slope_deg{0.0};
    /// 最小可通行净宽（米）= 车宽 + 安全余量。膨胀半径由它反推，不照抄外部参数。
    double min_passable_width{0.0};
    /// 能否上高地。哨兵为 false：高地在其视图中是硬障碍。
    bool can_climb_highland{false};
};

/// 通行档案：阵营 × 能力，由 `diesel_robots` 侧的 YAML 提供，本仓只消费。
struct TraversalProfile {
    Side side{Side::kRed};
    Capability capability{};
};

/// 导航目标。
struct NavGoal {
    Pose2 pose{};
    /// 到点容差（米）。
    double xy_tolerance{0.15};
    /// 是否要求到达时的朝向；全向底盘通常为 false。
    bool require_heading{false};
};

/// 导航状态机。
enum class NavStatus {
    kIdle = 0,
    kPlanning,
    kFollowing,
    kRecovering,
    kReached,
    kFailed,
};

/// 失败原因。失败必须带原因，不许只说"失败"。
enum class NavError {
    kNone = 0,
    /// 搜索空间耗尽或目标不可达。
    kNoPath,
    /// 起点落在地图之外。
    kStartOutsideMap,
    /// 目标落在致命代价格上（禁行区、高地、动态障碍等）。
    kGoalInLethal,
    /// 进度检查超时：卡住了。
    kProgressStalled,
    /// 恢复行为已用尽。
    kRecoveryExhausted,
    /// 输入非法（位姿超期、地图版本不匹配等）。
    kInvalidInput,
};

/// 全局路径：一串位姿，按行进顺序排列，含起点与终点。
using Path = std::vector<Pose2>;

/// 动态障碍：只按几何消费。
/// @note 敌我识别属 `diesel_vision`，本仓不解释语义，因此没有 label 字段。
struct DynamicObstacle {
    Vec2 center{};
    /// 保守外接半径（米）。
    double radius{0.0};
    /// 时间戳（纳秒）。过期障碍按"未知"处理。
    int64_t stamp_ns{0};
};

/// 底盘指令：全向三轴。
struct VelocityCommand {
    double vx{0.0};
    double vy{0.0};
    double omega{0.0};
};

/// 把错误码转成可读字符串（日志与状态上报用）。
inline const char* to_string(NavError error) {
    switch (error) {
    case NavError::kNone:
        return "none";
    case NavError::kNoPath:
        return "no_path";
    case NavError::kStartOutsideMap:
        return "start_outside_map";
    case NavError::kGoalInLethal:
        return "goal_in_lethal";
    case NavError::kProgressStalled:
        return "progress_stalled";
    case NavError::kRecoveryExhausted:
        return "recovery_exhausted";
    case NavError::kInvalidInput:
        return "invalid_input";
    }
    return "unknown";
}

/// 把状态转成可读字符串。
inline const char* to_string(NavStatus status) {
    switch (status) {
    case NavStatus::kIdle:
        return "idle";
    case NavStatus::kPlanning:
        return "planning";
    case NavStatus::kFollowing:
        return "following";
    case NavStatus::kRecovering:
        return "recovering";
    case NavStatus::kReached:
        return "reached";
    case NavStatus::kFailed:
        return "failed";
    }
    return "unknown";
}

/// 把兵种转成可读字符串（日志与状态上报用）。
inline const char* to_string(UnitKind kind) {
    switch (kind) {
    case UnitKind::kSentinel:
        return "sentinel";
    case UnitKind::kInfantry:
        return "infantry";
    case UnitKind::kHeavy:
        return "heavy";
    }
    return "unknown";
}

} // namespace nav
} // namespace diesel

#endif // DIESEL__NAV__TYPES_HPP_
