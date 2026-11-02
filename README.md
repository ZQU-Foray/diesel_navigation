# foray_navigation

> **L4** · owner 角色：导航

静态地图规划 + 动态障碍规避 + 全向控制器

基于机内预置地图规划，**不用在线 SLAM**。

---

## 现状：框架已落地（2026-10-28）

本仓当前交付的是**寻路框架**：公共接口 + 核心模块 + 独立测试。

| 模块 | 内容 | 测试 |
|---|---|---|
| `include/foray/nav/types.hpp` | 几何、阵营、**兵种（`UnitKind`）**、能力档案、目标、状态、错误码、路径、指令 | `types_test` |
| `include/foray/nav/costmap_view.hpp` | 代价图只读快照 + 代价编码（偏好 / 自由 / 膨胀 / 未知 / 致死） | — |
| `include/foray/nav/planner.hpp` | 全局规划器接口 + `PlanningOptions`（预算） | — |
| `include/foray/nav/controller.hpp` | 局部控制器接口（到达判定不在控制器内） | — |
| `src/grid_map.cpp` | PGM（P5/P2）+ YAML 元数据 → 栅格；world↔grid | `grid_map_test` |
| `src/costmap.cpp` | 静态 / 禁行 / 膨胀 / 动态 四层代价图 | `costmap_test` |
| `src/astar_planner.cpp` | A\*（8 邻接、禁斜穿犄角、视线捷径化、偏好代价） | `astar_planner_test` |
| `src/pursuit_controller.cpp` | 纯追踪（全向、曲率与到点降速、幅值与变化率限幅） | `pursuit_controller_test` |

**未实现（后续）**：恢复行为与状态机、端到端跑通、地图资产（PGM + 区域表）、与上游（`foray_localization` / `foray_vision` / `foray_robots`）的接线。

## 快速开始

本仓只依赖标准库与 CMake，**不需要 ROS**：

```bash
cmake -S . -B ~/.cache/foray-navigation-build -DCMAKE_BUILD_TYPE=Release
cmake --build ~/.cache/foray-navigation-build -j"$(nproc)"
ctest --test-dir ~/.cache/foray-navigation-build --output-on-failure
```

> 本仓位于 SMB 共享盘时构建目录必须放本地盘（共享盘不支持符号链接）。

在 ROS 2 工作空间中也可以 `colcon build`（`package.xml` 为 `build_type=cmake`），但核心库不依赖 ROS。

## 依赖

| 类型 | 依赖 |
|---|---|
| **自研仓** | 上游：`foray_interfaces`（X2）、`foray_platform`（L1/L2）、`foray_localization`（L3） |
| **第三方** | CMake ≥ 3.16、C++17 |

## 接口

> 本仓当前对外是 **C++ 库**，不是 ROS 节点；ROS 适配层后续加入。

| 头文件 | 命名空间 | 公共类型 / 函数 | 契约 |
|---|---|---|---|
| `foray/nav/types.hpp` | `foray::nav` | `Vec2` `Pose2` `Side` **`UnitKind`** `Capability` `TraversalProfile` `NavGoal` `NavStatus` `NavError` `Path` `DynamicObstacle` `VelocityCommand` | 失败必须带原因；动态障碍只按几何消费，**不做敌我识别**；兵种只换数值、不改接口 |
| `foray/nav/costmap_view.hpp` | `foray::nav` | `CostmapView` | 只读快照；代价**数值越小越划算**；`data()/stride()` 供热路径直接寻址 |
| `foray/nav/planner.hpp` | `foray::nav` | `GlobalPlanner` `PlanningOptions` | 纯函数式；代价必须当**边权重**用；必须遵守预算，不许阻塞 |
| `foray/nav/controller.hpp` | `foray::nav` | `LocalController` | 有状态；输出前必须限幅；**到达判定由状态机负责** |

## 纪律

- **只做寻路**：敌我识别归 `foray_vision`，定位与地图归 `foray_localization`，机间通信本赛季不允许；
- **激光只参与定位**，不进代价图、不做地形分析；
- 不用在线 SLAM；地图是离线资产，随赛季版本化；
- 动态障碍只按几何消费，**不解释语义**；
- **兵种接口预留**：`Capability::kind` 已有哨兵 / 步兵 / 重装三个取值，但本仓**只消费哨兵**的数值；步兵与重装只是把数值换成它们的，接口不变（2026-11-02，评审意见）；
- 依赖方向见 `.foray-layer`；本仓不得依赖 `foray_robots`。

## 上下游

| 方向 | 对象 |
|---|---|
| **上游**（本仓依赖谁） | `foray_interfaces` · `foray_platform` · `foray_localization` |
| **下游**（谁依赖本仓） | `foray_robots` · `foray_ws` |

## 设计文档

- [寻路框架设计](docs/nav_architecture.md)——职责边界、代价图分层、语义层、通行档案、验收矩阵
- 决策记录见 [decision.md](decision.md)；计划与进度见 [plan.md](plan.md)

## 参考

- [算法结构](https://github.com/ZQU-Foray/foray_docs/blob/main/algorithm_structure.md)
- [仓库结构](https://github.com/ZQU-Foray/foray_docs/blob/main/repository_structure.md)
- [组织贡献指南](https://github.com/ZQU-Foray/.github/blob/main/CONTRIBUTING.md)
