# 导航寻路框架设计

> 范围：本仓的**寻路（规划 + 控制）骨架**。不包含定位、地图构建、动态障碍感知。
> 场地数据来源：`RoboMaster 2027 高校联盟赛比赛规则手册 V1.0.0（20261015）`，
> 下文所有尺寸均注页码，代码中的同名常量必须带同样的注释。
>
> **状态（2026-11-04）：本文写的是设计意图，不是"已实现清单"。**
> 已经落地的是 M1–M4（接口、地图、代价图、A\*、纯追踪）；
> **M5 恢复与状态机、M6 端到端、ROS 适配层都还没写**——文中用现在时描述这些部分时，
> 请按"打算这样做"读，不要当现成能力。

## 1. 要解决的三个问题

| 问题 | 手段 | 本文对应章节 |
|---|---|---|
| 去哪 | 预置地图上的**全局路径** | §6 地图与语义层 · §7 全局寻路 |
| 怎么绕 | **代价图**（静态 + 禁行 + 膨胀 + 动态） | §5 代价图 |
| 到了没 | **局部控制** + 到点判定 + 恢复 | §8 局部控制 · §9 恢复 |

## 2. 范围边界（明确不做）

- **不做在线 SLAM**：地图是随赛季版本化的离线资产。
- **激光只用于定位**，不参与代价图、不做地形分析
  （依据 `foray_docs/algorithm_structure.md` §1.4/§1.5）。
- **不自建障碍感知**：动态障碍由 `diesel_vision` 的 `WorldSnapshot` 注入，本仓只消费。
- **不做机间通信与协同避让**（2027 赛季约束，见 §2.1）。
- **不定义 ROS 话题**：核心是纯 C++，ROS 适配层后续另加。

### 2.1 职责边界：本仓只负责寻路

本仓**只做寻路**——从哪儿到哪儿、怎么绕过去。下列事项各有归属模块，本仓不实现、不假设：

| 事项 | 归属 | 本仓的接口约定 |
|---|---|---|
| 敌我识别（装甲颜色） | `diesel_vision` | 不在本仓职责内；只按几何消费障碍 |
| 动态障碍检测 | `diesel_vision` | 只消费几何：位置、外接半径、时间戳 |
| 定位与地图 | `diesel_localization` | 只消费位姿与地图资产 |
| 机间通信 / 协同分工 | 本赛季不允许 | 不假设任何共享状态 |

因为不用机间通信、也不解释敌我，动态障碍层采取**纯几何**取向：

- 动态物体**一律占格、不可穿**，不分类别；
- 间隙由 `min_passable_width` 决定，不靠"它会动开"；
- 恢复只有「等 / 绕 / 退 / 上报」，没有"请求让路"；
- 三台车的分工只能写进赛前配置（例如哨兵守哪条走廊），不是运行时商量。

> 好处：本仓可完全离线回放、单测——既不依赖其它车的在线状态，
> 也不依赖上游检测质量，验收指标只衡量路径与控制。

## 3. 领域模型

```
WorldSnapshot (L3, 只读)          MapAsset (离线)
   ├─ 自身位姿 Pose2               ├─ occupancy 栅格（PGM + YAML）
   └─ 动态障碍 ObstacleSet         └─ 语义区域 RegionTable
            │                              │
            └──────────┬───────────────────┘
                       ▼
              TraversalProfile（阵营 × 兵种能力）
                       ▼
                  CostmapView（只读快照）
                       ▼
   GlobalPlanner ──► Path ──► LocalController ──► VelocityCommand
                       │
                       └──► NavStatus（状态机 + 恢复）
```

### 3.1 核心类型（草案，落地时以头文件为准）

```cpp
struct Vec2   { double x, y; };              // 米，场地坐标系
struct Pose2  { double x, y, yaw; };         // 米 / 弧度

enum class Side { kRed, kBlue };

// 兵种能力：只描述"这台车能不能"，不描述"想不想"
struct Capability {
    double max_slope_deg;       // 最大爬坡角
    double min_passable_width;  // 最小可通行净宽（= 车宽 + 安全余量）
    bool   can_climb_highland;  // 能否上高地
};

struct TraversalProfile {
    Side       side;
    Capability capability;
};

struct NavGoal {
    Pose2  pose;
    double xy_tolerance;   // 到点容差，米
    bool   require_heading;
};

enum class NavStatus {
    kIdle, kPlanning, kFollowing, kRecovering, kReached, kFailed
};

enum class NavError {            // 失败必须带原因，不许只说"失败"
    kNone, kNoPath, kStartOutsideMap, kGoalInLethal,
    kProgressStalled, kRecoveryExhausted
};

// 动态障碍：本仓只用几何，不解释语义（敌我识别属 diesel_vision）
struct DynamicObstacle {
    Vec2      center;
    double    radius;   // 保守外接半径
    Timestamp stamp;    // 过期障碍按"未知"处理
};

struct VelocityCommand { double vx, vy, omega; };   // 全向三轴
```

## 4. 契约

| 方向 | 内容 | 约束 |
|---|---|---|
| 输入 | `Pose2` + `Timestamp`（自身位姿，来自 `diesel_localization`） | 位姿必须带时间戳与置信度，超期不用 |
| 输入 | `ObstacleSet`（动态障碍，来自 `diesel_vision`） | 每个障碍带时间戳；过期障碍按"未知"处理 |
| 输入 | `MapAsset` + `TraversalProfile` | 地图版本随赛季绑定，不匹配直接失败 |
| 输入 | `NavGoal` | 目标落在致命代价格 → 立即 `kGoalInLethal` |
| 输出 | `VelocityCommand` | 幅值限幅 + 变化率限幅；**指令超时归零由执行器侧兜底**（L2 急停硬路径） |
| 输出 | `NavStatus` + `NavError` | 每次状态跃迁可观测 |

> 关键纪律：**规划与控制都不许自己"救"**。到不了就报 `kNoPath`，卡住就报 `kProgressStalled`，
> 由上层（L4 局部决策 / L5）决定下一步。

## 5. 代价图

分层，逐层叠加，只读快照，**永不就地修改**：

| 层 | 来源 | 说明 |
|---|---|---|
| 静态层 | `MapAsset` occupancy | 未知空间按**不可通行**处理（保守） |
| 禁行层 | 语义区域 + `TraversalProfile` | 对方补给区、我方不可达区域（如哨兵的上高地） |
| 膨胀层 | 静态层 + 禁行层 | 按 `min_passable_width` 推膨胀半径，**不是照抄 0.7 m** |
| 动态层 | `ObstacleSet` | 接口先留，第一版不接；只按几何占格，不分类别（§2.1） |

**膨胀半径的推法**（关键，避免"照抄参数把隧道堵死"）：

```
inflation_radius = max(0.5 × min_passable_width, 0.5 × 车宽 + 安全余量)
```

写进配置而不是常量；每个默认值必须注明"来源 / 待实测"。

**保守余量**：规则手册规定场地尺寸误差 **±10%**、结构角度误差 **±3°**（P.22），
地图与代价图按 **+10% 余量**处理边沿，不许贴着理论尺寸规划。

## 6. 地图与语义层

### 6.1 2027 赛季 3V3 场地数据（红线：改动必须同步页码）

| 项 | 值 | 来源 |
|---|---|---|
| 战场 | 12000 × 8000 mm | 规则手册 P.22 |
| 表面 | 木结构 + 2 mm PVC 地胶，部分结构有金属保护层 | P.22 |
| 补给区（兼启动区） | 3500 × 2500 mm | P.24 图 3-6 |
| 工作台 | 750 × 650 mm | P.24 图 3-6 |
| 高地 | 4000 × 2000 mm，高 400 mm（剖面两级各 200） | P.25 图 3-8 |
| 高地增益点平台 | 1800 mm | P.25 图 3-8 |
| 高地金属板 | 350 mm 宽 | P.25 图 3-8 |
| 隧道 | 位于补给区旁（图 3-9 只标增益点，**净宽待查制作规范手册**） | P.26 图 3-9 |
| 控制区 | 3 组：红方高地 / 中央 / 蓝方高地 | P.27 图 3-10 |
| 尺寸误差 | ±10%；结构角度 ±3°；<100 mm 结构 ±5 mm | P.22 |

> 待补：隧道净宽与净高、高地坡道坡度、工作台高度（`机器人制作规范手册`）。

### 6.2 语义区域表（不是像素，是有名字的区域）

决策层说"去中央控制区"，不该说"去 (6.0, 4.0)"。因此地图资源里带一张**区域表**：

```yaml
map_id: rmul2027_3v3
resolution: 0.05
origin: [0.0, 0.0, 0.0]     # 场地原点，对齐规则定义原点（不是开机位置）
regions:
  - {name: red_supply,      kind: supply,    side: red,  polygon: [...]}
  - {name: blue_supply,     kind: supply,    side: blue, polygon: [...]}
  - {name: red_highland_ctrl, kind: control, side: red,  polygon: [...]}
  - {name: central_ctrl,    kind: control,   side: neutral, polygon: [...]}
  - {name: highland,        kind: highland,  side: neutral, polygon: [...], height_m: 0.4}
  - {name: tunnel_red,      kind: tunnel,    side: red,  polygon: [...]}
  # ... 增益点逐个列
```

`kind` 决定代价语义：`supply/control/tunnel` 是可去目标，`forbidden` 是禁行，
`highland` 要按 `capability.can_climb_highland` 决定可通行与否。

### 6.3 用静态语义层替代在线地形分析（删除 `terrain_analysis` 之后）

原 Nav2 栈用 `terrain_analysis` 在线判断"这块地能不能上"。本仓不用激光做地形，
所以这件事必须**前置到地图资产**：

- 高地的坡道区：标 `ramp`，给坡度值，按 `max_slope_deg` 判可通行；
- 高地平台区：标 `highland`，按 `can_climb_highland` 判；
- 台阶边沿：标 `lethal`（两级 200 mm 台阶，轮式底盘不可直接下）。

**代价**：地图一变就得重标（可接受——地图是赛季资产，不是实时数据）。
**收益**：本仓不需要地形感知，可单测、可回放。

## 7. 通行档案：只有哨兵走自主导航

**2027 决策：本仓只服务哨兵。** 步兵和重装是人为操控的，不走自主导航——
所以不需要为它们生成视图，兵种维度先收窄成"哨兵一种能力档案"（字段留着，将来要加再说）。

同一张地图，**红蓝两侧的可通行域不同**：

| 视图 | 差异来源 | 例子 |
|---|---|---|
| 红方 × 哨兵 | 对方补给区禁行（P.26 图 3-7）＋ 哨兵不上高地 | 高地整块致死 ⇒ 战场被切成左右两条走廊 |
| 蓝方 × 哨兵 | 中心对称 | 由同一张地图镜像生成 |

> **哨兵不考虑上高地**（队伍决定）。因此哨兵视图里高地是硬障碍，
> 中央高地把战场切成两条走廊——**这是哨兵寻路的主要拓扑特征**，测试用例必须覆盖。
> 坡道与高地平台只作为"不可通行"标注存在，不做爬坡判定。

通行档案放 YAML（`diesel_robots` 侧），本仓只消费它，不写死兵种判断。

## 8. 全局寻路

第一版：**A\***（8 邻接，禁止斜穿犄角）。

- 场地 12 × 8 m，0.05 m/格 ⇒ 240 × 160 = 38 400 格，A\* 完全够用；
- 代价从 `CostmapView` 读，越界/未知/致死一律不可入；
- 结果做**视线捷径化**（line-of-sight shortcut）：能直连就跳过中间点，
  比引入完整 smoother 便宜，也够用；
- 失败原因必须区分：起点在图外 / 目标在致死格 / 搜索空间耗尽。

**θ\* 作为候选，不默认启用**：它的收益是少一层折线平滑，但本场地小、A\* 已足够；
只有在实测显示"折线导致跟随抖动"时才替换（见 `decision.md`）。

## 9. 局部控制

全向底盘，三轴独立：

```
前视点  = 路径上距我 lookahead 的点（速度自适应：lookahead = clamp(k·v, min, max)）
vx, vy  = 朝前视点的方向 × 期望速度
omega   = 可选（enable_rotation）；默认 0，全向底盘靠平移绕行
```

`VelocityScheduler`（**借鉴思路，自己实现**）统一处理降速，三条都要有：

| 项 | 规则 | 依据 |
|---|---|---|
| 曲率降速 | 曲率 > `curvature_min` 开始降，> `curvature_max` 降到 `reduction_ratio` | 北极熊 2025 实测参数（值待实测） |
| 到点减速 | 距目标 < `approach_velocity_scaling_dist` 线性降到 `min_approach_linear_velocity` | 同上 |
| 变化率限幅 | 速度缩放因子每周期变化 ≤ `max_velocity_scaling_factor_rate` | 同上 |

外加：**到点判定**（`xy_tolerance`；`require_heading` 时才判 yaw）与
**进度检查**（`required_movement` / `movement_time_allowance`，超时 → `kProgressStalled`）。

## 10. 恢复

按顺序尝试，每步有超时，全部失败 → `kRecoveryExhausted`：

1. 清空代价图缓存（重建快照）；
2. 原地旋转一圈（刷新"我以为堵了"的误判）；
3. 沿反方向退到最近自由格（借鉴 `BackUpFreeSpace` 的思路：退到"看得见的路"）；
4. 重新规划一次（限次数）；
5. 上报失败，**不强行穿行**。

## 11. 参数策略

- 全部进 schema，两级：`shared.yaml`（全兵种）+ `<robot>.yaml`（兵种差异），
  复用 `diesel_platform` 的 `param`；
- 借来的默认值必须标来源，例：
  ```cpp
  // 来源：pb_omni_pid_pursuit_controller 2025 实机配置；我方车体未标定，待实测
  double translation_kp = 3.0;
  ```
- **魔数不得裸奔**：场地尺寸、容差、膨胀半径，全部带页码或标定日期。

## 12. 验收

### 12.1 测试矩阵（每个用例一个可执行测试）

| 用例 | 期望 |
|---|---|
| 空场直线 | 路径 ≈ 直线，长度 ≤ 1.05 × 欧氏距离 |
| L 形拐角 | 不切死角，离障碍 ≥ 膨胀半径 |
| 我方补给区 | 可进入（补给点） |
| 对方补给区 | 不可进入，绕行 |
| **高地** | 高地整块不可通行，必须走两侧走廊（哨兵视图唯一一种） |
| 隧道 | 能通过，路径最小间隙 ≥ `min_passable_width` |
| 目标在致死格 | `kGoalInLethal`，不规划 |
| 全封闭 | `kNoPath`，且耗时有上界（不许卡死） |
| 动态障碍挡死（后续） | 触发恢复，恢复到上限后 `kRecoveryExhausted` |
| 障碍堵在隧道（后续） | 无通信、不协商：等待超时 → 绕行 → 仍不通则 `kRecoveryExhausted` |
| 指令限幅 | 任何输入下 |v| ≤ 上限、变化率 ≤ 上限 |

### 12.2 指标

| 指标 | 目标 |
|---|---|
| 单次规划耗时（240×160 格） | < 10 ms |
| 路径长度 | ≤ 1.15 × 同图 Dijkstra 最优 |
| 控制周期 | 20 Hz（可配置） |
| 最小障碍间隙 | ≥ 膨胀半径，且报数 |

## 13. 借 / 不借清单

| 类别 | 内容 | 处置 |
|---|---|---|
| **借结论** | 曲率降速、到点减速、变化率限幅、积分限幅 | 自己实现，用单测证明"有它不甩、没它会甩" |
| **借结论** | 恢复顺序（清图 → 转 → 退 → 重规划） | 自己实现 |
| **借结构** | 行为树式的"重规划 + 恢复"流程 | 用轻量状态机实现，不引入 BehaviorTree.CPP |
| **借参数** | Kp/Ki/Kd、前视距离、速度上限、膨胀半径 | 当默认值，**逐条标"待实测"** |
| **不借** | Nav2 三层代价图 + action server + pluginlib 那一整套 | 太重，与本仓"不依赖 ROS 才能测"冲突 |
| **不借** | `IntensityVoxelLayer`（激光强度当障碍层） | 违反"激光只定位" |
| **不借** | `terrain_analysis` / `terrain_analysis_ext` | 已定删除，用静态语义层替代 |

许可：参考的 `pb_omni_pid_pursuit_controller` / `pb_nav2_plugins` 为 Apache-2.0，
**思路可借鉴、代码不得整段搬运**；确需引用片段时保留版权头并在 `NOTICE` 记录。

## 14. 以后

| 项 | 触发条件 |
|---|---|
| θ\* 替换 A\* | 实测折线跟随抖动明显 |
| 动态障碍层接 `WorldSnapshot` | `diesel_vision` 契约冻结 |
| ROS 适配层 | `diesel_robots` 装配根就绪 |
| 机间协同避让 | **本赛季不允许机间通信，不做**；赛制放开后再议 |
