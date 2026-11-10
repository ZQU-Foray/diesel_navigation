# 开发计划

## 任务列表

### 寻路框架（M0–M6，设计见 `docs/nav_architecture.md`）

- [x] **M0 设计**：契约、代价图分层、语义层、通行档案、验收矩阵
- [x] **M1a 接口**：`types.hpp` / `costmap_view.hpp` / `planner.hpp` / `controller.hpp`（header-only，零警告；已按评审修正契约）
- [x] **M1b 地图**：`GridMap`（PGM P5/P2 + YAML 元数据 + 三态代价 + world↔grid），独立测试全绿
- [x] **M2 代价图**：静态层 + 禁行层（多边形）+ 膨胀层（倒角距离变换，半径由 `min_passable_width` 推出）+ 动态层占格
- [x] **M3 全局寻路**：A\*（8 邻接、禁斜穿犄角）+ 视线捷径化 + 失败原因分类；支持**偏好代价**（隧道优先）
- [x] **M4 局部控制**：纯追踪（全向）+ 曲率降速 + 到点降速 + 幅值/变化率限幅
- [x] **M4.5 引擎（2026-11-07）**：`NavEngine` 把「规划 → 跟踪 → 判到达 → 偏离/停滞重规划」串成主循环；
  安全层 / 护栏层两级膨胀、起终点兜底、安全自检；`nav_demo`、`nav_trace` 两个工具；
  2027 真图资产 `maps/rmul_2027/`；压测 105 组随机 + 94 组贴墙起始，全部到达、0 帧压致死格
- [ ] **M5 恢复与状态机**：清图 → 旋转 → 后退 → 重规划 → 上报。
  **部分已内建**：偏离/停滞重规划、「先退再跟」都在 `NavEngine` 里；**独立的恢复状态机尚未做**
- [ ] **M6 端到端**：2027 3V3 地图上跑通「补给区 → 中央控制区」、路径可视化。
  **离线压测已做**（随机起终点全到达、前后对比图见 `docs/figures/`）；
  **轨迹出图已做**（`nav_plot`，2026-11-10）；但**任务级端到端（按区域起终点跑完整任务）仍未做**

### 原基线清理（承接 fork 分支）

- [ ] 清理 fork：删除 terrain_analysis 等已定删除项
- [x] 把导航参数抽出为 `shared.yaml` + `<robot>.yaml` 两级（2026-11-10：
      `config/shared.yaml` + `config/rmul_2027.yaml`，`params` 模块 + `params_test`，两个工具都能 `--params/--robot`）
- [ ] 接 `foray_localization` 的位姿与地图（替代当前直连）
- [ ] 接 `foray_vision` 的动态障碍（替代自建障碍源）
- [ ] 拆出 `foray_robots` 装配根，本仓只留算法

## 当前 Agent State

`NAV_ENGINE_DONE` —— 框架 + 引擎都落地了：接口 / 地图 / 代价图 / A\* / 隧道偏好 / 控制 / 主循环，
六组测试全绿，2027 真图压测全到达。M5 的独立恢复状态机、M6 的任务级端到端、ROS 适配与上游接线仍属后续实现。

> 2026-11-10：文档与代码对齐（README / plan / tree / decision），代码未改。

## Before Snapshot

commit hash:   （本次提交；引擎侧见 `c323eb1`）
branch:        feat/nav-skeleton
modified files: 2026-11-10 仅文档（`README.md`、`plan.md`、`tree.md`、`decision.md`）
risk level:    L1（纯文档，无代码改动）

## 模糊点与待确认项

- **2027 赛季不允许机间通信**（已落进设计与决策）：本仓不假设共享状态，
  动态障碍只按几何消费；敌我识别属 `foray_vision`，本仓不参与。
  仍需队里定：**三台车抢同一条走廊**时如何赛前分工（YAML 写死）
- **隧道尺寸已核（2026-11-10）**：通道净宽约 0.82 m、穿过的高地墙厚约 0.44 m、净高 250 mm；
  **高地坡道坡度、工作台高度**仍未查到。
- **车体尺寸已给（2026-11-10）**：哨兵 **593 × 593 × 243 mm** → 半宽 296.5 mm、外接半径 419 mm。
  `robot_radius` 取 **0.32 m**（半宽 + 余量）；标定参数（最大速度、最大爬坡角）仍待实测。
  **注意**：车高 243 mm 对隧道净高 250 mm 只剩 7 mm，实车量超了就得把隧道从区域表删掉。
- 地图资产已有场地栅格图（`maps/rmul_2027/`），但**区域表与语义层（坡道 / 高地 / 台阶边沿 / 补给区）尚未标注**
- 本仓接口尚未冻结，任务清单为**方向性**的，落地顺序以 `foray_docs/algorithm_structure.md` §9 演进阶段为准
- 依赖的自研仓尚未建齐，跨仓任务需等对方 `README.md` 明确接口后再启动

## Vector Backend Status

Backend: Markdown
Status:  ready
Environment: 仓库内 Markdown 文档（无外部向量后端）
Index: 本仓 `README.md` / `tree.md` / `decision.md`
Initialization: 2026-02-19
Commit: （首次提交）

## Acceptance Criteria

- [x] 本仓能独立 `colcon build`——2026-11-07 CI 的 `build-ros2` 作业里，
      `colcon build` / `colcon test` 都是真跑过（不是 skipped），四个检查全绿
- [x] CI 绿灯（2026-11-02：「All checks have passed」，4 个检查）
- [x] 每个新增模块带独立可执行测试（`grid_map_test` / `costmap_test` / `astar_planner_test` /
      `pursuit_controller_test` / `nav_engine_test` / `types_test`）
- [ ] PR #1 合入：**只差有人 Approve 与实机验证**（代码不再改动）
