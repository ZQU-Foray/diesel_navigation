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

// ROS 2 适配层：把 NavEngine 包成一个节点。**核心库不依赖 ROS**，这一层只做话题/参数/时间。
//
// 订阅：
//   map        nav_msgs/OccupancyGrid       （transient_local，晚到的节点也能拿到）
//   odom       nav_msgs/Odometry            （取位姿；TF 以后再说）
//   goal_pose  geometry_msgs/PoseStamped    （RViz 的 "2D Goal Pose" 默认话题）
// 发布：
//   cmd_vel    geometry_msgs/Twist
//
// 参数：
//   params_file   通用参数 YAML（同 config/shared.yaml）
//   robot_file    车种覆盖 YAML（同 config/rmul_2027.yaml）
//   regions_file  区域表（隧道）；空表示不用
//   control_rate  控制周期（Hz），默认 20
//
// 设计取舍：位姿直接吃 odom，不做 TF 树——定位模块（diesel_localization）还没接，
// 先保证「地图 + 位姿 + 目标 → 速度指令」这条最小闭环。接定位时只换这一处。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "diesel/nav/costmap.hpp"
#include "diesel/nav/nav_engine.hpp"
#include "diesel/nav/params.hpp"
#include "diesel/nav/regions.hpp"

namespace foray {
namespace nav {

class NavNode : public rclcpp::Node {
  public:
    NavNode() : rclcpp::Node("diesel_navigation") {
        params_file_ = declare_parameter<std::string>("params_file", "");
        robot_file_ = declare_parameter<std::string>("robot_file", "");
        regions_file_ = declare_parameter<std::string>("regions_file", "");
        const double rate = declare_parameter<double>("control_rate", 20.0);

        // 参数：命令行给了 YAML 就以文件为准，否则用编译期默认值。
        NavParams loaded{};
        std::string error;
        bool ok = true;
        if (!params_file_.empty()) {
            ok = load_params(params_file_, robot_file_, &loaded, &error);
        } else if (!robot_file_.empty()) {
            ok = load_params_file(robot_file_, &loaded, &error);
        }
        if (!ok) {
            RCLCPP_FATAL(get_logger(), "参数加载失败：%s", error.c_str());
            throw std::runtime_error(error);
        }
        engine_ = std::make_unique<NavEngine>(loaded.astar, loaded.pursuit);
        engine_->options() = loaded.engine;

        if (!regions_file_.empty()) {
            std::vector<RectRegion> regions;
            if (!load_regions_file(regions_file_, &regions, &error)) {
                RCLCPP_FATAL(get_logger(), "区域表加载失败：%s", error.c_str());
                throw std::runtime_error(error);
            }
            engine_->set_preferred_regions(regions);
            RCLCPP_INFO(get_logger(), "区域表：%zu 块（隧道标为偏好）", regions.size());
        }
        RCLCPP_INFO(get_logger(), "车体半径 %.2f m，限速 %.2f m/s", loaded.engine.robot_radius,
                    loaded.pursuit.max_linear);

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // 地图用 transient_local：晚启动的节点也能拿到上一张。
        const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", map_qos, [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { on_map(*msg); });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(*msg); });
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 1,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { on_goal(*msg); });

        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate));
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   [this]() { on_timer(); });
    }

  private:
    /// 占据栅格 → 代价图：-1 未知、>=65 视为墙（阈值与 nav2 的默认一致）。
    void on_map(const nav_msgs::msg::OccupancyGrid& msg) {
        const auto& info = msg.info;
        std::string error;
        if (!map_.init(static_cast<int>(info.width), static_cast<int>(info.height), info.resolution,
                       Vec2{info.origin.position.x, info.origin.position.y}, &error)) {
            RCLCPP_ERROR(get_logger(), "地图初始化失败：%s", error.c_str());
            return;
        }
        for (int y = 0; y < static_cast<int>(info.height); ++y) {
            for (int x = 0; x < static_cast<int>(info.width); ++x) {
                const int8_t value = msg.data[static_cast<size_t>(y) * info.width + x];
                uint8_t cost = kCostFree;
                if (value < 0) {
                    cost = kCostUnknown;
                } else if (value >= 65) {
                    cost = kCostLethal;
                }
                map_.set_cost(x, y, cost);
            }
        }
        has_map_ = true;
        RCLCPP_INFO(get_logger(), "地图就绪：%d x %d，%.3f m/格", map_.width(), map_.height(),
                    map_.resolution());
        if (has_goal_ && has_pose_) {
            replan();
        }
    }

    void on_odom(const nav_msgs::msg::Odometry& msg) {
        const auto& p = msg.pose.pose.position;
        const auto& q = msg.pose.pose.orientation;
        pose_.x = p.x;
        pose_.y = p.y;
        // yaw（绕 z 轴）
        pose_.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        has_pose_ = true;
    }

    void on_goal(const geometry_msgs::msg::PoseStamped& msg) {
        goal_.pose.x = msg.pose.position.x;
        goal_.pose.y = msg.pose.position.y;
        goal_.pose.yaw = std::atan2(2.0 * (msg.pose.orientation.w * msg.pose.orientation.z +
                                           msg.pose.orientation.x * msg.pose.orientation.y),
                                    1.0 - 2.0 * (msg.pose.orientation.y * msg.pose.orientation.y +
                                                 msg.pose.orientation.z * msg.pose.orientation.z));
        has_goal_ = true;
        RCLCPP_INFO(get_logger(), "新目标：(%.2f, %.2f)", goal_.pose.x, goal_.pose.y);
        if (has_map_ && has_pose_) {
            replan();
        } else {
            RCLCPP_WARN(get_logger(), "还缺%s，先记下目标", !has_map_ ? "地图" : "位姿");
        }
    }

    void replan() {
        if (engine_ == nullptr) {
            return;
        }
        const NavError error = engine_->set_goal(map_, goal_, pose_);
        if (error != NavError::kNone) {
            RCLCPP_ERROR(get_logger(), "规划失败 err=%d", static_cast<int>(error));
            return;
        }
        RCLCPP_INFO(get_logger(), "规划成功：%zu 个路径点", engine_->path().size());
    }

    void on_timer() {
        geometry_msgs::msg::Twist cmd{}; // 默认零速
        if (engine_ != nullptr && has_map_ && has_pose_ && has_goal_) {
            VelocityCommand command{};
            const NavError error = engine_->step(map_, pose_, kPeriod, &command);
            cmd.linear.x = command.vx;
            cmd.linear.y = command.vy;
            cmd.angular.z = command.omega;

            if (error != NavError::kNone && engine_->status() == NavStatus::kFailed) {
                RCLCPP_ERROR(get_logger(), "导航失败 err=%d", static_cast<int>(error));
                has_goal_ = false;
                cmd = geometry_msgs::msg::Twist{};
            } else if (engine_->status() == NavStatus::kReached) {
                RCLCPP_INFO(get_logger(), "到达目标");
                has_goal_ = false;
                cmd = geometry_msgs::msg::Twist{};
            }
        }
        cmd_pub_->publish(cmd);
    }

    static constexpr double kPeriod = 0.05; ///< 与 control_rate 默认值一致（20 Hz）

    std::string params_file_;
    std::string robot_file_;
    std::string regions_file_;
    std::unique_ptr<NavEngine> engine_;
    Costmap map_;
    NavGoal goal_;
    Pose2 pose_{};
    bool has_map_{false};
    bool has_pose_{false};
    bool has_goal_{false};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace nav
} // namespace foray

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<diesel::nav::NavNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("diesel_navigation"), "节点启动失败：%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
