建图时

Global Options
Fixed Frame: map
Grid
Reference Frame: map
Plane: XY
RobotModel
Description Topic: /robot_description
TF
不需要填 topic，直接启用
Map
Topic: /map
LaserScan
Topic: /scan
Odometry
Topic: /odometry/filtered
如果你想看底盘原始轮速，也可以改成 /odom_wheel，但我更建议看融合后的 /odometry/filtered
导航时

Global Options
Fixed Frame: map
Grid
Reference Frame: map
RobotModel
Description Topic: /robot_description
TF
启用
Map
Topic: /map
LaserScan
Topic: /scan
Odometry
Topic: /odometry/filtered
2D Pose Estimate
用来给 AMCL 设初始位姿
2D Goal Pose
用来发导航目标