# 目录结构说明

## 目录结构

```
.
├── README.md                本仓定位、接口、上下游
├── AGENTS.md                AI 协作规范
├── plan.md                  开发计划与进度
├── decision.md              关键工程决策日志
├── tree.md                  本文件
├── CODEOWNERS               Review 自动分派
├── .foray-layer             所属层与依赖边界（CI 校验）
├── .clang-format            组织 C/C++ 风格（4 空格、指针靠左、列宽 100）
├── .gitignore
├── CMakeLists.txt           库 foray_navigation + 六个测试 + 两个工具（C++17，零警告）
├── package.xml              build_type=cmake，供 colcon 识别
├── docs/
│   ├── nav_architecture.md  寻路框架设计：契约、代价图、语义层、通行档案、验收
│   └── figures/
│       └── nav-before-after.png  同一任务改造前后轨迹 + 到达数演进（供 PR 效果展示）
├── maps/
│   └── rmul_2027/           2027 3V3 场地图（512×352、0.025 m/格）
│       ├── rmul_2027.pgm
│       └── rmul_2027.yaml
├── config/
│   ├── shared.yaml          通用参数：键清单 + 默认值口径（哪些标「待实测」）
│   ├── rmul_2027.yaml       哨兵 / 2027 赛季覆盖层（只写跟 shared 不一样的）
│   └── rmul_2027.regions.txt 区域表：隧道（tunnel，可覆盖墙）/ 偏好走廊（preferred）
├── include/foray/nav/       公共接口（header-only，不依赖 ROS）
│   ├── types.hpp            几何/阵营/兵种/能力/目标/状态/错误码/路径/指令
│   ├── costmap_view.hpp     代价图只读快照接口 + 代价编码
│   ├── planner.hpp          全局规划器接口
│   ├── controller.hpp       局部控制器接口
│   ├── grid_map.hpp         栅格地图：PGM + YAML 元数据 + 坐标换算
│   ├── costmap.hpp          分层代价图：静态/禁行/膨胀/动态
│   ├── astar_planner.hpp    A* 全局寻路实现
│   ├── pursuit_controller.hpp  纯追踪控制器实现
│   ├── nav_engine.hpp       导航主循环：规划 → 跟踪 → 判到达 → 重规划
│   ├── params.hpp           参数集合 + 扁平 YAML 两级加载 + 合法性检查
│   └── regions.hpp          区域表：隧道 / 偏好走廊（可覆盖原图致死格）
├── src/
│   ├── grid_map.cpp         栅格地图：PGM 读入 + YAML 元数据 + 坐标换算
│   ├── costmap.cpp          分层代价图：静态/禁行/膨胀/动态
│   ├── astar_planner.cpp    A* 全局寻路（8 邻接、禁斜穿犄角、视线捷径化）
│   ├── pursuit_controller.cpp  纯追踪控制（全向、限幅、曲率与到点降速）
│   ├── nav_engine.cpp       主循环 + 两级膨胀（安全层 / 护栏层）+ 起终点兜底 + 安全自检
│   ├── params.cpp           扁平 YAML 解析：未知键报错、数值校验、文件:行 定位
│   └── regions.cpp          区域表解析与应用：tunnel 可覆盖墙，preferred 不行
├── apps/
│   ├── nav_demo.cpp         演示：内置地图或真实 PGM 上跑通「规划→跟踪→到达」
│   ├── nav_trace.cpp        逐帧诊断：位姿/指令/净空/状态，可导 CSV
│   └── nav_plot.cpp         轨迹出图：CSV + 地图 → PPM（速度染色、起终点标记）
├── ros/
│   └── nav_node.cpp         ROS 2 适配层（可选构建）：map + odom + goal_pose → cmd_vel
├── test/                    独立可执行测试（不依赖测试框架）
│   ├── grid_map_test.cpp    地图读入与坐标换算
│   ├── costmap_test.cpp     代价图分层、膨胀幂等、过期障碍、细窄禁行带
│   ├── astar_planner_test.cpp  A* 用例（含隧道偏好）
│   ├── pursuit_controller_test.cpp  控制器用例
│   ├── nav_engine_test.cpp  主循环：到点、拒收致死目标、重规划、NaN/dt 边界
│   ├── params_test.cpp      参数：默认值、两级覆盖、错键/坏值/越界必须报错
│   ├── regions_test.cpp     区域表：解析、两种类型语义（tunnel 可开墙、preferred 不行）
│   └── types_test.cpp       兵种接口预留（只换数值、不改接口）
└── .github/
    └── workflows/
        └── ci.yml           复制自组织 CI 模板
```

> 本仓已有源码（`include/` `src/` `apps/` `test/` `maps/`），随代码变更同步更新本文件。

## 记录约束

MUST NOT 记录以下内容：

- `build/`、`install/`、`log/`、`.git/` 等依赖与产物目录
- 临时文件、缓存文件、日志文件
