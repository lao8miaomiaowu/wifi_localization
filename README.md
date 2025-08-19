# Wi-Fi 指纹定位 ROS2 节点（C++）

> 基于 **ROS2 Humble** 的 Wi-Fi 指纹定位系统，通过扫描周围 Wi-Fi 信号强度（RSSI），利用 ONNX 模型实时推断机器人在二维地图中的坐标，并将结果在 **RViz** 中可视化。

---

## 🌟 功能特性

- ✅ **实时扫描** 指定 SSID 列表的 RSSI（使用 `nmcli`，无需 root）
- ✅ **ONNX 推理** 输出二维坐标 `(x, y)`
- ✅ **ROS2 标准接口** 发布 `/wifi_pose` 与 `/wifi_path`
- ✅ **RViz 可视化** 一键查看定位轨迹
- ✅ **参数化配置** SSID、扫描周期均可通过 ROS 参数修改

---

## 📦 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/<你的用户名>/wifi_localization_cpp.git
cd wifi_localization_cpp
```

### 2. 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. 运行节点

```bash
ros2 run wifi_localization_cpp wifi_localizer_node \
  --ros-args -p target_ssids:="['BISTU', 'BISTU-802.1X']"
```

### 4. 打开 RViz 可视化

```bash
rviz2
```

- 将 **Fixed Frame** 设为 `map`
- **Add → By topic** 添加 `/wifi_pose`（Pose） 与 `/wifi_path`（Path）

---

## ⚙️ 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `target_ssids` | `string[]` | `["BISTU", "BISTU-802.1X"]` | 需要扫描的 SSID 列表 |
| `scan_period_sec` | `double` | `2.0` | 扫描与推理周期（秒） |

---

## 📁 目录结构

```
wifi_localization_cpp/
├── src/
│   └── wifi_localizer_node.cpp    # 主节点源码
├── include/                       # 头文件（如有）
├── launch/
│   └── wifi_rviz.launch.py        # 一键启动节点 + RViz
├── models/
│   └── CNN-net.onnx               # 预训练指纹定位模型
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 🔧 开发记录

| 问题 | 解决方案 |
|------|----------|
| **权限不足无法扫描** | 改用 `nmcli` 获取 RSSI，无需 root |
| **接口名错误** | 通过 `ip link` 确认为 `wlo1` |
| **正则解析失败** | 匹配 `nmcli -t -f ssid,signal` 输出 |
| **坐标固定不变** | 修复 RSSI 读取逻辑后，坐标随信号实时变化 |
| **RViz 不显示** | 手动启动 `rviz2` 并添加 `/wifi_pose` 与 `/wifi_path` |

---

## 🤝 贡献

欢迎提交 **Issue** 或 **Pull Request**！  
如果你有新的模型、更多 SSID 或改进建议，请一起完善这个项目。

---

## 📄 许可证

MIT © 2024 <你的名字>

---

**如有疑问，欢迎在 GitHub Issues 中讨论！**
