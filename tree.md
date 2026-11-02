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
├── .gitignore
├── docs/
│   └── nav_architecture.md  寻路框架设计：契约、代价图、语义层、通行档案、验收
├── include/foray/nav/       公共接口（header-only，不依赖 ROS）
│   ├── types.hpp            几何/阵营/能力/目标/状态/错误码/路径/指令
│   ├── costmap_view.hpp     代价图只读快照接口 + 代价编码
│   ├── planner.hpp          全局规划器接口
│   └── controller.hpp       局部控制器接口
├── src/
│   ├── grid_map.cpp         栅格地图：PGM 读入 + YAML 元数据 + 坐标换算
│   ├── costmap.cpp          分层代价图：静态/禁行/膨胀/动态
│   ├── astar_planner.cpp    A* 全局寻路（8 邻接、禁斜穿犄角、视线捷径化）
│   └── pursuit_controller.cpp  纯追踪控制（全向、限幅、曲率与到点降速）
├── test/
│   ├── grid_map_test.cpp    地图独立测试（不依赖测试框架）
│   ├── costmap_test.cpp     代价图分层与膨胀测试
│   ├── astar_planner_test.cpp  A* 用例（含隧道偏好）
│   ├── pursuit_controller_test.cpp  控制器用例
│   └── types_test.cpp       兵种接口预留（只换数值、不改接口）
├── CMakeLists.txt           库 foray_navigation（C++17，零警告）
├── .clang-format            组织 C/C++ 风格（4 空格、指针靠左、列宽 100）
├── package.xml              build_type=cmake，供 colcon 识别
└── .github/
    └── workflows/
        └── ci.yml           复制自组织 CI 模板
```

> 本仓已有源码（`include/` `src/` `test/`），随代码变更同步更新本文件。

## 记录约束

MUST NOT 记录以下内容：

- `build/`、`install/`、`log/`、`.git/` 等依赖与产物目录
- 临时文件、缓存文件、日志文件
