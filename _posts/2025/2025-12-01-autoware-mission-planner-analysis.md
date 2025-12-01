---
layout: post
title: "Autoware Mission Planner 深度解析：嵌入式系统中的自动驾驶任务规划"
author: allen.mkj
date: 2025-12-01
categories: [嵌入式]
tags: [Autoware, 自动驾驶, 任务规划, C++, ROS2, 嵌入式系统]
excerpt: 深入分析 Autoware Mission Planner 的核心机制，探讨嵌入式系统中自动驾驶任务规划的技术实现。
cover: /assets/images/posts/2025-12-01-autoware-mission-planner-analysis/cover.png
---

## Mission Planner 

### 核心状态定义

```cpp
enum class RouteState {
    UNSET,     // 路径未设置
    SET,       // 路径已设置，正在执行
    CHANGING,  // 路径正在修改
    ARRIVED    // 已到达目标
};
```

### 1. 地图与定位数据处理

#### 地图数据接收
```cpp
void MissionPlanner::on_map(const HADMapBin::ConstSharedPtr msg)
{
    map_ptr_ = msg;
}
```

#### 里程计数据处理
```cpp
void MissionPlanner::on_odometry(const Odometry::ConstSharedPtr msg)
{
    odometry_ = msg;

    // 在 SET 状态下进行到达检测
    if (state_.state == RouteState::Message::SET) {
        PoseStamped pose;
        pose.header = odometry_->header;
        pose.pose = odometry_->pose.pose;
        if (arrival_checker_.is_arrived(pose)) {
            change_state(RouteState::Message::ARRIVED);
        }
    }
}
```

**设计亮点**：
- 采用回调机制实时更新车辆状态
- 状态检查与到达检测解耦，提高系统响应性
- 使用智能指针管理内存，避免内存泄漏

### 2. 路径设置机制

Mission Planner 提供两种路径设置方式：

#### 基于路径段的路径设置
```cpp
void MissionPlanner::on_set_route(
  const SetRoute::Service::Request::SharedPtr req, 
  const SetRoute::Service::Response::SharedPtr res)
{
    // 前置条件检查
    if (state_.state != RouteState::Message::UNSET) {
        throw component_interface_utils::ServiceException(
            ResponseCode::ERROR_ROUTE_EXISTS, "The route is already set.");
    }
    
    // 创建新路径
    const auto route = create_route(req);
    
    // 验证路径有效性
    if (route.segments.empty()) {
        throw component_interface_utils::ServiceException(
            ResponseCode::ERROR_PLANNER_FAILED, "The planned route is empty.");
    }
    
    // 更新系统状态
    change_route(route);
    change_state(RouteState::Message::SET);
    res->status.success = true;
}
```

#### 基于路径点的路径设置
`on_set_route_points()` 函数提供更灵活的路径输入方式，允许开发者通过一系列路径点来定义行驶路线。

### 3. 动态重路由系统

#### 安全重路由机制
```cpp
bool MissionPlanner::check_reroute_safety(
  const LaneletRoute & original_route, const LaneletRoute & target_route)
{
    // 前置条件检查
    if (original_route.segments.empty() || target_route.segments.empty() || 
        !map_ptr_ || !odometry_) {
        return false;
    }

    // 车辆静止时允许重路由
    const auto current_velocity = odometry_->twist.twist.linear.x;
    if (current_velocity < 0.01) {
        return true;
    }

    // 查找共同路径段
    const auto start_idx_opt = find_common_segment_start(original_route, target_route);
    if (!start_idx_opt.has_value()) {
        return false;
    }

    // 计算安全距离
    double accumulated_length = calculate_common_route_length(original_route, target_route);
    
    // 安全长度检查
    const double safety_length = std::max(
        current_velocity * reroute_time_threshold_,    // 时间阈值 × 速度
        minimum_reroute_length_                         // 最小安全距离
    );
    
    return accumulated_length > safety_length;
}
```

**安全检查逻辑**：
1. **静止优先**：车辆静止时允许任意重路由
2. **连续性检查**：确保新旧路径有足够的共同段
3. **动态安全距离**：根据当前速度计算最小安全距离
4. **多重验证**：结合时间阈值和距离阈值

### 4. 紧急处理机制（MRM）

MRM（Minimal Risk Maneuver）是 Mission Planner 中的关键安全机制，用于处理紧急情况下的路径规划。

#### MRM 路径设置
```cpp
void MissionPlanner::on_set_mrm_route(
  const SetMrmRoute::Service::Request::SharedPtr req,
  const SetMrmRoute::Service::Response::SharedPtr res)
{
    // 状态检查
    if (!planner_->ready() || !odometry_) {
        throw component_interface_utils::ServiceException(
            ResponseCode::ERROR_PLANNER_UNREADY, "The planner is not ready.");
    }

    const auto prev_state = state_.state;
    change_state(RouteState::Message::CHANGING);

    // 规划紧急路径
    const auto new_route = create_route(req);

    if (new_route.segments.empty()) {
        change_state(prev_state);
        throw component_interface_utils::ServiceException(
            ResponseCode::ERROR_PLANNER_FAILED, "Failed to plan a new route.");
    }

    // 智能路径选择
    if (mrm_route_) {
        if (check_reroute_safety(*mrm_route_, new_route)) {
            change_mrm_route(new_route);
            res->status.success = true;
        } else {
            change_mrm_route(*mrm_route_);
            res->status.success = false;
        }
    } else if (!normal_route_) {
        change_mrm_route(new_route);
        change_state(RouteState::Message::SET);
        res->status.success = true;
    } else {
        // 与正常路径比较安全性
        if (check_reroute_safety(*normal_route_, new_route)) {
            change_mrm_route(new_route);
            res->status.success = true;
        } else {
            change_route(*normal_route_);
            res->status.success = false;
        }
    }
    change_state(RouteState::Message::SET);
}
```

### 5. 到达检测机制

到达检测是 Mission Planner 的关键功能，确保车辆准确到达目标位置：

```cpp
bool ArrivalChecker::is_arrived(const PoseStamped & pose) const
{
    if (!goal_with_uuid_) return false;
    
    // 1. 坐标系一致性检查
    if (goal.header.frame_id != pose.header.frame_id) return false;
    
    // 2. 距离检查
    if (distance_ < tier4_autoware_utils::calcDistance2d(pose.pose, goal.pose)) 
        return false;
    
    // 3. 角度检查
    const double yaw_diff = tier4_autoware_utils::normalizeRadian(yaw_pose - yaw_goal);
    if (angle_ < std::fabs(yaw_diff)) return false;
    
    // 4. 车辆停止状态检查
    return vehicle_stop_checker_.isVehicleStopped(duration_);
}
```

**四重检测机制**：
1. **坐标系验证**：确保位姿数据在同一坐标系下
2. **距离阈值**：车辆与目标的平面距离
3. **角度阈值**：车辆朝向与目标朝向的差异
4. **停止状态**：车辆必须保持稳定停止状态

## 系统工作流程

![Mission Planner 工作流程图](/assets/images/posts/2025-12-01-autoware-mission-planner-analysis/mission_planner.png)
png
### 初始化阶段
1. **系统启动**：加载配置参数，初始化状态机
2. **数据等待**：等待地图数据和车辆定位信息
3. **服务就绪**：路径规划器准备就绪

### 路径设置阶段
1. **接收请求**：通过 ROS2 服务接收路径设置请求
2. **路径规划**：调用底层路径规划算法生成行驶路径
3. **状态更新**：将系统状态设置为 SET

### 路径执行阶段
1. **持续监控**：实时监控车辆位置和速度
2. **动态调整**：处理目标点修改和重路由请求
3. **安全检查**：确保所有操作满足安全要求

### 到达处理阶段
1. **到达检测**：四重机制验证到达条件
2. **状态更新**：将系统状态设置为 ARRIVED
3. **任务完成**：准备接收下一个任务

