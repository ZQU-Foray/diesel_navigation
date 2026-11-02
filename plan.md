# 开发计划

## 任务列表

### 寻路框架（M0–M6，设计见 `docs/nav_architecture.md`）

- [x] **M0 设计**：契约、代价图分层、语义层、通行档案、验收矩阵
- [x] **M1a 接口**：`types.hpp` / `costmap_view.hpp` / `planner.hpp` / `controller.hpp`（header-only，零警告；已按评审修正契约）
- [x] **M1b 地图**：`GridMap`（PGM P5/P2 + YAML 元数据 + 三态代价 + world↔grid），独立测试全绿
- [x] **M2 代价图**：静态层 + 禁行层（多边形）+ 膨胀层（倒角距离变换，半径由 `min_passable_width` 推出）+ 动态层占格
- [x] **M3 全局寻路**：A\*（8 邻接、禁斜穿犄角）+ 视线捷径化 + 失败原因分类；支持**偏好代价**（隧道优先）
- [x] **M4 局部控制**：纯追踪（全向）+ 曲率降速 + 到点降速 + 幅值/变化率限幅
- [x] **评审跟进（2026-11-02）**：① `UnitKind` 兵种接口预留（哨兵 / 步兵 / 重装，本期只用哨兵）② 补 `.clang-format` 并重排全部源码，修 `lint-c`
- [ ] **M5 恢复与状态机**：清图 → 旋转 → 后退 → 重规划 → 上报（**后续实现，不属于框架阶段**）
- [ ] **M6 端到端**：2027 3V3 地图上跑通「补给区 → 中央控制区」，路径可视化（**后续实现**）

### 原基线清理（承接 fork 分支）

- [ ] 清理 fork：删除 terrain_analysis 等已定删除项
- [ ] 把导航参数抽出为 `shared.yaml` + `<robot>.yaml` 两级
- [ ] 接 `foray_localization` 的位姿与地图（替代当前直连）
- [ ] 接 `foray_vision` 的动态障碍（替代自建障碍源）
- [ ] 拆出 `foray_robots` 装配根，本仓只留算法

## 当前 Agent State

NAV_FRAMEWORK_DONE（框架到此为止：接口 / 地图 / 代价图 / A\* / 隧道偏好 / 控制，五组测试全绿；M5 恢复、M6 端到端属后续实现，不在框架阶段做）

## Before Snapshot

commit hash:   （本次提交）
branch:        feat/nav-skeleton
modified files: 新增 `include/foray/nav/*.hpp`、`CMakeLists.txt`、`package.xml`；`tree.md`、`plan.md` 同步
risk level:    L1（纯文档，无代码）

## 模糊点与待确认项

- **2027 赛季不允许机间通信**（已落进设计与决策）：本仓不假设共享状态，
  动态障碍只按几何消费；敌我识别属 `foray_vision`，本仓不参与。
  仍需队里定：**三台车抢同一条走廊**时如何赛前分工（YAML 写死）
- **隧道净宽 / 净高、高地坡道坡度、工作台高度**尚未查到，在 `机器人制作规范手册` 里，
  补齐前 M2 的膨胀半径只能按估计值
- **车体尺寸与标定参数未定**（车宽、最大速度、最大爬坡角），所有借来的默认值标「待实测」
- 地图资产（PGM + 区域表）尚无产出，M1 用的第一张图需先建
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

- [ ] 本仓能独立 `colcon build`（无代码时跳过）——**待验**：本地没有 ROS 2 环境，
      CI 里的 `build-ros2` 作业在无 `package.xml` 时会跳过，跳过也算"成功"，所以不能拿绿灯当证据
- [x] CI 绿灯（2026-11-02：PR #1「All checks have passed」，4 个检查）
- [x] 每个新增模块带独立可执行测试（`grid_map_test` / `costmap_test` / `astar_planner_test` / `pursuit_controller_test` / `types_test`）
