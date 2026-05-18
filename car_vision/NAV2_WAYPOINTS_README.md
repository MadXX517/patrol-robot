# RViz 点选多点导航使用说明

本文档说明 `car_vision` 中独立的多点导航节点 `nav2_waypoints` 的使用方式。

## 功能说明

`nav2_waypoints` 不会替换原来的单点导航功能。

- 原单点/固定点导航入口仍然是：

```bash
ros2 run car_vision nav2_goals
```

- 新增 RViz 点选多点导航入口是：

```bash
ros2 run car_vision nav2_waypoints
```

多点导航使用 RViz 的 `Publish Point` 工具收集点位，不使用 `2D Goal Pose`。

## RViz 中应该使用哪个工具

请使用工具栏里的红色图钉：

```text
Publish Point
```

不要使用绿色箭头：

```text
2D Goal Pose
```

二者区别如下：

- `2D Goal Pose` 会直接向 Nav2 发送单点导航目标，新的目标会抢占旧目标。
- `Publish Point` 只发布 `/clicked_point`，车不会立刻移动，适合用来收集多个点。

## 使用步骤

1. 启动雷达导航和 RViz。

2. 启动多点导航节点：

```bash
ros2 run car_vision nav2_waypoints
```

3. 在 RViz 中选择 `Publish Point`。

4. 在地图上依次点击多个可到达点。

   每点击一次，节点会把该点加入 waypoint 队列，并在 `/waypoints` 上发布编号可视化标记。

5. 点完后，在终端发送开始命令：

```bash
ros2 topic pub --once /nav2_waypoints/command std_msgs/msg/String "{data: 'start'}"
```

机器人会按照点击顺序依次导航。

## 控制命令

撤销最后一个点：

```bash
ros2 topic pub --once /nav2_waypoints/command std_msgs/msg/String "{data: 'undo'}"
```

清空所有点：

```bash
ros2 topic pub --once /nav2_waypoints/command std_msgs/msg/String "{data: 'clear'}"
```

取消正在执行的多点导航：

```bash
ros2 topic pub --once /nav2_waypoints/command std_msgs/msg/String "{data: 'cancel'}"
```

开始执行当前点队列：

```bash
ros2 topic pub --once /nav2_waypoints/command std_msgs/msg/String "{data: 'start'}"
```

## 注意事项

- 点击点必须位于地图中可规划、可通行的区域。
- 多点导航过程中再次点击 `Publish Point` 会被忽略，避免临时改动正在执行的任务。
- 如果需要重新选点，请先 `cancel`，再 `clear`，然后重新点击。
- 当前 waypoint 默认只使用位置，朝向设置为 `w=1.0`。如果需要精确终点朝向，应继续使用单点导航或扩展本节点。

## 构建

修改后需要重新构建 `car_vision`：

```bash
colcon build --packages-select car_vision
source install/setup.bash
```
