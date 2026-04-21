目录结构
drone_estimation/
├── config/
│   └── ekf.yaml            # 双 EKF 节点参数配置文件
├── launch/
│   └── drone_sim.launch.py # 一键启动仿真、EKF 及分析节点
├── scripts/
│   ├── gt.py               # 真值降频与噪声注入脚本 (模拟GPS)
│   └── plot_trajectory.py   # 实时轨迹对比与误差分析绘图脚本
└── urdf/
    └── drone.urdf.xacro    # 无人机(立方体)模型及传感器配置

    启动仿真与状态估计
    ros2 launch drone_estimation drone_sim.launch.py

    控制立方体移动
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

    启动轨迹绘制脚本   先进入scripts文件夹
    python3 plot_trajectory.py