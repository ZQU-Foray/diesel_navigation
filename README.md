# foray_navigation

> **L4** · owner 角色：导航

静态地图规划 + 动态障碍规避 + 全向控制器

基于机内预置地图规划，**不用在线 SLAM**。

---

## 现状：框架 + 引擎已落地（2026-11-07）

本仓当前交付的是**寻路框架 + 导航引擎**：公共接口、核心模块、把「规划→跟踪→判到达→重规划」
串起来的主循环，以及两个命令行工具和 2027 赛季真实场地图。全部**不依赖 ROS**，可独立构建与测试。

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
| `src/nav_engine.cpp` | **主循环**：规划 → 跟踪 → 判到达 → 偏离/停滞重规划；安全层与护栏层两级膨胀、起终点兜底、安全自检 | `nav_engine_test` |
| `include/foray/nav/params.hpp` + `src/params.cpp` | **参数出代码**：扁平 YAML 两级覆盖（`shared` → `<robot>`），未知键与越界值直接报错 | `params_test` |
| `include/foray/nav/regions.hpp` + `src/regions.cpp` | **区域表**：把「图上画成墙、但人能过」的隧道补回代价图（`tunnel` 可覆盖墙，`preferred` 不行） | `regions_test` |
| `apps/nav_demo.cpp` | 4 m × 3 m 人造带墙图上跑通「规划→跟踪→到达」；也可读真实 PGM | — |
| `apps/nav_trace.cpp` | 逐帧诊断：位姿/指令/净空/状态，可导 CSV、可只看卡住段 | — |
| `apps/nav_plot.cpp` | 把 `nav_trace` 的 CSV 画到地图上，输出 PPM：单条按速度染色，多条按顺序配色（`--csv` 可给多次） | — |
| `ros/nav_node.cpp` | **ROS 2 适配层**（可选构建）：`map`+`odom`+`goal_pose` → `cmd_vel`；核心库仍不依赖 ROS | 见下文「ROS 2 适配层」 |
| `maps/rmul_2027/` | 2027 3V3 场地图（512 × 352、0.025 m/格） | — |

**未实现（后续）**：

- **独立恢复状态机（M5 完整形态）**——重规划与「先退再跟」已内建在 `NavEngine` 里，但没有独立的恢复行为层；
- **端到端任务（M6）**：补给区 → 中央控制区的完整路线与可视化；
- **ROS 上游接线**：TF / `foray_localization` 的位姿（现在节点直接吃 `odom`）、`foray_vision` 的动态障碍、`foray_robots` 的装配根；
- **地图语义层与区域表**（坡道 / 高地 / 台阶边沿 / 补给区）尚未标注（隧道已标）；
- **fork 基线清理**（删 `terrain_analysis`、接 `foray_localization` / `foray_vision` / `foray_robots`）。

### 2026-11-07 引擎落地（NavEngine + 真图压测）

- **两级膨胀**：规划与安全自检用「按车体半径膨胀过的图」，让路径自己走中间；刹车另用一张更窄的
  **护栏图**（0.6 × 半径，贴墙时按实际净空自适应，下限 0.4 × 半径）——不能用「我已经在里面的余量」来判我。
- **起终点兜底**：`clear_disc()` 清掉车 / 终点周围一圈被膨胀盖住的格子；但**原图上是墙或未知的格子绝不清**
  （清了会在薄墙上打洞——实测 33 条路径里 13 条穿墙）。
- **原点必须继承**：安全图沿用原图 `origin()`；派生图与原图原点不一致时**直接拒绝规划**
  （此前写死 `(0,0)`，整张障碍布局偏了 0.4 m）。
- **恢复**：偏离超阈值 / 停滞超时重规划，次数用尽才失败；前方被判死时朝「净空变大」的方向退一小步，
  先退再跟。
- **边界防护**：`dt ≤ 0`、NaN 位姿一律 `kInvalidInput` —— 上游一帧坏数据不许被判成「到了」。
- **验证**：2027 真图 105 组随机任务 + 94 组贴墙起始，**全部到达、0 帧压在致死格**；
  `docs/figures/nav-before-after.png` 是同一任务的前后轨迹与到达数演进（47 → 57 → 64 → 93 → 103 → 105）。

**2026-11-04 按代码审计修正**（审计由队友做，34 条）：

- 控制器前视点改为**从车在路径上的位置起算**——此前前视点固定从路径起点算，20 m 直线只能走到 1.365 m 就来回撞；
- 到点停车走**变化率限幅**，不再一帧从 2.5 m/s 跌到 0；
- 代价图 `inflate()` **幂等**（此前每调一次可通行区缩一圈）；
- 平滑阶段也守 `deadline_ns` 预算；
- PGM 的 `maxval` 参与归一化；坏文件先校验长度再分配；
- A\* 输出路径补**中间点朝向**；`1.02` 平滑余量落成有出处的常量；
- `cost_scaling` 方向修正（越大衰减越快）；未知格参与距离变换；
- 动态障碍支持**过期**（`set_obstacle_timeout_ns`）；亚格宽度的禁行带不再漏；
- 补 `omega` 与到点朝向对齐（`PursuitParams::align_heading_at_goal`，默认关闭）。

## 快速开始

本仓只依赖标准库与 CMake，**不需要 ROS**：

```bash
cmake -S . -B ~/.cache/foray-navigation-build -DCMAKE_BUILD_TYPE=Release
cmake --build ~/.cache/foray-navigation-build -j"$(nproc)"
ctest --test-dir ~/.cache/foray-navigation-build --output-on-failure
```

> 本仓位于 SMB 共享盘时构建目录必须放本地盘（共享盘不支持符号链接）。

在 ROS 2 工作空间中也可以 `colcon build`（`package.xml` 为 `build_type=cmake`），但核心库不依赖 ROS。

### 跑一遍

```bash
# 演示：内置地图（4 m × 3 m，中间一堵墙留一个 1.4 m 的口）
<build>/nav_demo

# 读真实地图
<build>/nav_demo --yaml maps/rmul_2027/rmul_2027.yaml --goal-x 6.6 --goal-y 0.5

# 逐帧诊断：改控制环之前先看数据，别只看红绿
<build>/nav_trace --yaml maps/rmul_2027/rmul_2027.yaml --sx 1.9 --sy 5.1 --gx 6.6 --gy 0.5
<build>/nav_trace --yaml maps/rmul_2027/rmul_2027.yaml --sx 1.9 --sy 5.1 --gx 6.6 --gy 0.5 --csv trace.csv

# 把这一趟画成图：蓝=慢、红=快，绿点起点、红点终点（PPM，--scale 放大）
<build>/nav_plot --yaml maps/rmul_2027/rmul_2027.yaml --csv trace.csv --out trace.ppm --scale 2
```

> `<build>` 指上面 `cmake --build` 用的那个构建目录，例如 `~/.cache/foray-navigation-build`。

### 参数

参数**不再写死在代码里**（2026-11-10 起）：先读 `config/shared.yaml`（本仓通用），再叠加
`config/<robot>.yaml`（车种 / 赛季覆盖，只写不一样的那几条）。

```bash
<build>/nav_demo  --params config/shared.yaml --robot config/rmul_2027.yaml
<build>/nav_trace --yaml maps/rmul_2027/rmul_2027.yaml --sx 1.9 --sy 5.1 --gx 6.6 --gy 0.5 \
                  --params config/shared.yaml --robot config/rmul_2027.yaml
```

- 格式刻意保持**扁平**（`key: value`，`#` 起注释），不依赖第三方 YAML 库；
- **键名写错、数值不合法都会报错并指到「文件:行」**——静默忽略的代价是「调了半天，发现改的是另一个名字」；
- 每个键的默认值与来历（哪些是**待实测**的借用值）都写在 `config/shared.yaml` 的注释里。

### 区域表（隧道）

2027 场地有两条隧道，PGM 把它们画成实心黑柱（图中量到的墙厚约 0.44 m）——**图上看着就是墙**，
再按车体半径一膨胀，连柱子两侧的空地也糊成致死，路就彻底断了。

隧道真实尺寸（2026-11-10 核对）：**通道净宽约 0.82 m、穿过的高地墙厚约 0.44 m、净高 250 mm**；
哨兵车体 **593 × 593 × 243 mm** —— 宽度够（两侧各 103 mm），**高度只剩 7 mm**（这条软件管不了，
要实车量；超过 250 mm 就必须把隧道从区域表里删掉，不能骗车）。

这一层信息只能由人来标，写在 `config/rmul_2027.regions.txt`：

```
tunnel    3.00 7.05 4.00 7.87     # 上隧道：可覆盖原图上的「墙」
tunnel    8.00 0.12 9.00 0.95     # 下隧道
preferred ...                     # 只想「更想走」、不许覆盖墙的区域
```

```bash
<build>/nav_trace ... --regions config/rmul_2027.regions.txt
```

- `tunnel` 是**人的显式决定**：允许把原图上的致死格打开（未知格永远不打开）；
- 区域要**沿通道轴各放宽约 0.3 m**，否则隧道口仍被膨胀带封死（实测：只标柱子本身，路线一点不变）；
- 实测效果：横穿全图同一任务，**18.10 → 14.48 m**、9.5 → 7.1 s。

### ROS 2 适配层（可选）

`ros/nav_node.cpp` 把 `NavEngine` 包成一个节点。**核心库仍然不依赖 ROS**：CMake 里
`find_package(rclcpp QUIET)` 找不到就整个目标跳过，纯 CMake 构建照常可用。

| 方向 | 话题 | 类型 |
|---|---|---|
| 订阅 | `map` | `nav_msgs/OccupancyGrid`（`transient_local`，晚启动也能拿到） |
| 订阅 | `odom` | `nav_msgs/Odometry`（取位姿；**暂不接 TF 树**——定位模块还没接） |
| 订阅 | `goal_pose` | `geometry_msgs/PoseStamped`（RViz 的 2D Goal Pose） |
| 发布 | `cmd_vel` | `geometry_msgs/Twist`（`vx`/`vy`/`omega`） |

参数：`params_file`、`robot_file`、`regions_file`、`control_rate`（默认 20 Hz）。

```bash
source /opt/ros/humble/setup.bash && colcon build
./install/foray_navigation/lib/foray_navigation/nav_node \
  --ros-args -p params_file:=$PWD/config/shared.yaml \
             -p robot_file:=$PWD/config/rmul_2027.yaml \
             -p regions_file:=$PWD/config/rmul_2027.regions.txt
```

2026-11-10 在 Ubuntu 22.04 + Humble 上验证：`colcon build` 过；喂一张 4×3 m 带墙的地图 +
固定位姿 + 目标后，节点规划出 6 个路径点、`cmd_vel` 达到 2.5 m/s 限速（20 Hz，120 帧）。

> **还没接的**：TF / `foray_localization` 的位姿（现在直接吃 `odom`）、`foray_vision` 的动态障碍、
> 参数动态重配。这些等上游接口定了再接，不然接口会返工。

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
