# EKF 与 TF 归属

TF 归属规则：
- `odom -> base_footprint` 只由底盘 originbot_base 发布，启动时保持 `pub_odom:=true`（默认值）。
- `map -> odom` 由 ekf_map.launch.py 中的 map_to_odom 节点发布。
- 不要同时运行 AMCL 与本套全局 EKF，否则会有两个 `map -> odom` 发布者。

推荐启动顺序：

```
$ ros2 launch originbot_bringup originbot.launch.py use_imu:=true pub_odom:=true
$ ros2 launch originbot_navigation ekf_map.launch.py
$ ros2 launch originbot_navigation nav_bringup_no_amcl.launch.py
```

可选：odom_ekf.launch.py 只输出 `/odometry/filtered`（odom 系），不再发布 TF。
它和 ekf_map.launch.py 都会发布 `/odometry/filtered`，不要同时运行。

ArUco 定位节点后续发布 `/aruco/pose`（map 系 PoseWithCovarianceStamped），
ekf_map.yaml 已预留 `pose0: /aruco/pose`。
