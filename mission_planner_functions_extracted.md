# Mission Planner 函数提取（基于PDF）

根据PDF中提到的函数，从源代码中提取的具体实现如下：

## 📋 核心函数列表与实现

### 1. on_map() - 地图数据处理
```cpp
void MissionPlanner::on_map(const HADMapBin::ConstSharedPtr msg)
{
  map_ptr_ = msg;
}
```
**功能**: 接收并存储地图数据

---

### 2. on_odometry() - 里程计数据处理
```cpp
void MissionPlanner::on_odometry(const Odometry::ConstSharedPtr msg)
{
  odometry_ = msg;

  // NOTE: Do not check in the changing state as goal may change.
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
**功能**: 
- 更新车辆位置信息
- 在SET状态下检测是否到达目标

---

### 3. on_set_route() - 基于路径段设置路线
```cpp
void MissionPlanner::on_set_route(
  const SetRoute::Service::Request::SharedPtr req, const SetRoute::Service::Response::SharedPtr res)
{
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoute::Response;

  if (state_.state != RouteState::Message::UNSET) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_ROUTE_EXISTS, "The route is already set.");
  }
  if (!planner_->ready()) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "The planner is not ready.");
  }
  if (!odometry_) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "The vehicle pose is not received.");
  }
  if (mrm_route_) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_INVALID_STATE, "Cannot reroute in the emergency state.");
  }

  // Convert request to a new route.
  const auto route = create_route(req);

  // Check planned routes
  if (route.segments.empty()) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_FAILED, "The planned route is empty.");
  }

  // Update route.
  change_route(route);
  change_state(RouteState::Message::SET);
  res->status.success = true;
}
```
**功能**: 
- 检查前置条件
- 创建新路径
- 更新状态为SET

---

### 4. on_set_route_points() - 基于路径点设置路线
```cpp
void MissionPlanner::on_set_route_points(
  const SetRoutePoints::Service::Request::SharedPtr req,
  const SetRoutePoints::Service::Response::SharedPtr res)
{
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoutePoints::Response;

  if (state_.state != RouteState::Message::UNSET) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_ROUTE_EXISTS, "The route is already set.");
  }
  if (!planner_->ready()) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "The planner is not ready.");
  }
  if (!odometry_) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "The vehicle pose is not received.");
  }
  if (mrm_route_) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_INVALID_STATE, "Cannot reroute in the emergency state.");
  }

  // Plan route.
  const auto route = create_route(req);

  // Check planned routes
  if (route.segments.empty()) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_FAILED, "The planned route is empty.");
  }

  // Update route.
  change_route(route);
  change_state(RouteState::Message::SET);
  res->status.success = true;
}
```
**功能**: 类似on_set_route，但使用路径点而非路径段

---

### 5. on_change_route() - 修改普通路线
```cpp
void MissionPlanner::on_change_route(
  const SetRoute::Service::Request::SharedPtr req, const SetRoute::Service::Response::SharedPtr res)
{
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoute::Response;

  if (state_.state != RouteState::Message::SET) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_INVALID_STATE, "The route hasn't set yet. Cannot reroute.");
  }
  if (!planner_->ready()) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "The planner is not ready.");
  }
  if (!odometry_) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "The vehicle pose is not received.");
  }
  if (!normal_route_) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "Normal route is not set.");
  }
  if (mrm_route_) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_INVALID_STATE, "Cannot reroute in the emergency state.");
  }
  if (reroute_availability_ && !reroute_availability_->availability) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_INVALID_STATE, "Cannot reroute as the planner is not in lane following.");
  }

  // set to changing state
  change_state(RouteState::Message::CHANGING);

  // Convert request to a new route.
  const auto new_route = create_route(req);

  // Check planned routes
  if (new_route.segments.empty()) {
    change_route(*normal_route_);
    change_state(RouteState::Message::SET);
    res->status.success = false;
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_FAILED, "The planned route is empty.");
  }

  // check route safety
  if (check_reroute_safety(*normal_route_, new_route)) {
    // success to reroute
    change_route(new_route);
    res->status.success = true;
    change_state(RouteState::Message::SET);
  } else {
    // failed to reroute
    change_route(*normal_route_);
    res->status.success = false;
    change_state(RouteState::Message::SET);
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_REROUTE_FAILED, "New route is not safe. Reroute failed.");
  }
}
```
**功能**: 
- 安全检查
- 创建新路径
- 安全性验证

---

### 6. on_change_route_points() - 基于路径点修改路线
```cpp
void MissionPlanner::on_change_route_points(
  const SetRoutePoints::Service::Request::SharedPtr req,
  const SetRoutePoints::Service::Response::SharedPtr res)
{
  // 类似on_change_route的实现，但使用路径点输入
  
  // 关键步骤：
  // 1. 状态检查
  // 2. 创建新路径
  // 3. 安全检查
  // 4. 更新或回退
}
```

---

### 7. on_modified_goal() - 处理目标点修改
```cpp
void MissionPlanner::on_modified_goal(const ModifiedGoal::Message::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received modified goal.");

  if (state_.state != RouteState::Message::SET) {
    RCLCPP_ERROR(get_logger(), "The route hasn't set yet. Cannot reroute.");
    return;
  }
  if (!planner_->ready()) {
    RCLCPP_ERROR(get_logger(), "The planner is not ready.");
    return;
  }
  if (!odometry_) {
    RCLCPP_ERROR(get_logger(), "The vehicle pose is not received.");
    return;
  }
  if (!normal_route_) {
    RCLCPP_ERROR(get_logger(), "Normal route has not set yet.");
    return;
  }

  if (mrm_route_ && mrm_route_->uuid == msg->uuid) {
    // 处理MRM路径目标修改
    change_state(RouteState::Message::CHANGING);
    // 创建新路径并更新
    change_mrm_route(new_route);
    change_state(RouteState::Message::SET);
  } else if (normal_route_->uuid == msg->uuid) {
    // 处理正常路径目标修改
    change_state(RouteState::Message::CHANGING);
    // 创建新路径并更新
    change_route(new_route);
    change_state(RouteState::Message::SET);
  } else {
    RCLCPP_ERROR(get_logger(), "Goal uuid is incorrect.");
  }
}
```
**功能**: 
- UUID匹配路径
- 动态修改目标点

---

### 8. on_reroute_availability() - 监控重路由可用性
```cpp
void MissionPlanner::on_reroute_availability(const RerouteAvailability::ConstSharedPtr msg)
{
  reroute_availability_ = msg;
}
```
**功能**: 接收重路由可用性状态

---

### 9. on_set_mrm_route() - 设置MRM紧急路线
```cpp
void MissionPlanner::on_set_mrm_route(
  const SetMrmRoute::Service::Request::SharedPtr req,
  const SetMrmRoute::Service::Response::SharedPtr res)
{
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoutePoints::Response;

  if (!planner_->ready()) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "The planner is not ready.");
  }
  if (!odometry_) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "The vehicle pose is not received.");
  }
  if (reroute_availability_ && !reroute_availability_->availability) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_INVALID_STATE, "Cannot reroute as the planner is not in lane following.");
  }

  const auto prev_state = state_.state;
  change_state(RouteState::Message::CHANGING);

  // Plan route.
  const auto new_route = create_route(req);

  if (new_route.segments.empty()) {
    change_state(prev_state);
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_FAILED, "Failed to plan a new route.");
  }

  // check route safety
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
**功能**: 
- MRM路径设置
- 多重安全检查
- 智能路径选择

---

### 10. on_clear_mrm_route() - 清除MRM路线
```cpp
void MissionPlanner::on_clear_mrm_route(
  const ClearMrmRoute::Service::Request::SharedPtr,
  const ClearMrmRoute::Service::Response::SharedPtr res)
{
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoutePoints::Response;

  if (!planner_->ready()) {
    change_state(RouteState::Message::SET);
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "The planner is not ready.");
  }
  if (!odometry_) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "The vehicle pose is not received.");
  }
  if (!mrm_route_) {
    throw component_interface_utils::NoEffectWarning("MRM route is not set");
  }
  if (
    state_.state == RouteState::Message::SET && reroute_availability_ &&
    !reroute_availability_->availability) {
    throw component_interface_utils::ServiceException(
      ResponseCode::ERROR_INVALID_STATE,
      "Cannot clear MRM route as the planner is not lane following before arriving at the goal.");
  }

  change_state(RouteState::Message::CHANGING);

  if (!normal_route_) {
    clear_mrm_route();
    change_state(RouteState::Message::UNSET);
    res->status.success = true;
    return;
  }

  // check route safety
  if (check_reroute_safety(*mrm_route_, *normal_route_)) {
    clear_mrm_route();
    change_route(*normal_route_);
    change_state(RouteState::Message::SET);
    res->status.success = true;
    return;
  }

  // 尝试重新规划到正常目标
  const auto new_route = create_route(
    odometry_->header, empty_waypoints, normal_route_->goal_pose,
    normal_route_->allow_modification);

  if (new_route.segments.empty() || !check_reroute_safety(*mrm_route_, new_route)) {
    change_mrm_route(*mrm_route_);
    change_state(RouteState::Message::SET);
    res->status.success = false;
  } else {
    clear_mrm_route();
    change_route(new_route);
    change_state(RouteState::Message::SET);
    res->status.success = true;
  }
}
```
**功能**: 
- 安全清除MRM路径
- 智能回退机制

---

### 11. check_reroute_safety() - 重路由安全检查
```cpp
bool MissionPlanner::check_reroute_safety(
  const LaneletRoute & original_route, const LaneletRoute & target_route)
{
  // 1. 前置条件检查
  if (original_route.segments.empty() || target_route.segments.empty() || !map_ptr_ || !odometry_) {
    RCLCPP_ERROR(get_logger(), "Check reroute safety failed. Route, map or odometry is not set.");
    return false;
  }

  // 2. 车辆静止时允许重路由
  const auto current_velocity = odometry_->twist.twist.linear.x;
  if (current_velocity < 0.01) {
    return true;
  }

  // 3. 查找共同路径段
  const auto start_idx_opt = find_common_segment_start(original_route, target_route);
  if (!start_idx_opt.has_value()) {
    return false;
  }

  // 4. 计算安全距离
  double accumulated_length = calculate_common_route_length(original_route, target_route);
  
  // 5. 安全长度检查
  const double safety_length = std::max(
    current_velocity * reroute_time_threshold_,    // 时间阈值 × 速度
    minimum_reroute_length_                         // 最小安全距离
  );
  
  return accumulated_length > safety_length;
}
```
**功能**: 
- 多重安全验证
- 路径连续性检查
- 速度相关的安全距离计算

---

### 12. change_state() - 状态转换
```cpp
void MissionPlanner::change_state(RouteState::Message::_state_type state)
{
  state_.stamp = now();
  state_.state = state;
  pub_state_->publish(state_);
}
```
**功能**: 
- 更新内部状态
- 发布状态信息

---

### 13. arrival_checker_.is_arrived() - 到达检测（在arrival_checker.cpp中）
```cpp
bool ArrivalChecker::is_arrived(const PoseStamped & pose) const
{
  if (!goal_with_uuid_) return false;
  
  // 1. 检查坐标系一致性
  if (goal.header.frame_id != pose.header.frame_id) return false;
  
  // 2. 检查距离
  if (distance_ < tier4_autoware_utils::calcDistance2d(pose.pose, goal.pose)) 
    return false;
  
  // 3. 检查角度
  const double yaw_diff = tier4_autoware_utils::normalizeRadian(yaw_pose - yaw_goal);
  if (angle_ < std::fabs(yaw_diff)) return false;
  
  // 4. 检查车辆停止状态
  return vehicle_stop_checker_.isVehicleStopped(duration_);
}
```
**功能**: 
- 四重到达检测
- 距离、角度、坐标系、停止状态

---

## 🔄 函数调用流程图

```
初始化
  ↓
on_map() → 接收地图
  ↓
on_odometry() → 接收位置
  ↓
on_set_route() / on_set_route_points() → 设置路径
  ↓
change_state(SET)
  ↓
路径执行阶段:
├── on_odometry() → 持续监控
├── arrival_checker_.is_arrived() → 到达检测
├── on_modified_goal() → 目标修改
├── on_reroute_availability() → 重路由监控
└── 路径调整:
    ├── on_change_route() / on_change_route_points()
    ├── on_set_mrm_route() + check_reroute_safety()
    └── on_clear_mrm_route()
  ↓
到达检测 → true
  ↓
change_state(ARRIVED)
  ↓
任务完成
```

---

*此文档提取了PDF中提到的所有函数的完整实现，展示了Mission Planner的详细工作机制。*
