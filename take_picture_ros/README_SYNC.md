# Take Picture ROS 时间同步配置说明

## 概述

本文档说明如何将 `take_picture_ros` 相机驱动与 `livox_ros_driver2` 的时间同步功能集成，替代原有的 `mvs_ros_driver`。

## 时间同步原理

系统使用**共享内存**机制实现时间同步：

1. **Livox雷达驱动** (`livox_ros_driver2`) 将同步的时间戳写入共享内存文件：`/home/<username>/timeshare`
2. **相机驱动**从该共享内存读取时间戳，用作图像消息的时间戳
3. 这样保证了雷达点云和相机图像使用相同的时间基准

### 共享内存数据结构

```cpp
struct time_stamp {
  int64_t high;  // 高位（未使用）
  int64_t low;   // 低位，单位：纳秒
};
```

时间戳计算：`time_seconds = low / 1000000000.0`

## 新增文件说明

### 1. camera_ros_driver_one_with_sync.cpp

这是修改后的相机驱动程序，新增以下功能：

- 打开并映射共享内存 `/home/<username>/timeshare`
- 从共享内存读取Livox的同步时间戳
- 支持通过参数 `enable_sync` 切换是否使用同步时间戳
- 如果共享内存不可用，自动回退到使用 `ros::Time::now()`

### 2. camera_with_sync.launch

启动文件，配置参数：

- `topic_name`: 图像发布话题名称
- `pipeline`: GStreamer管道配置
- `rate_hz`: 发布频率
- `enable_sync`: 是否启用时间同步（true/false）

## 使用步骤

### 1. 编译

```bash
cd ~/catkin_ws  # 替换为你的工作空间路径
catkin_make
# 或使用
catkin build take_picture_ros
```

### 2. 确保共享内存文件存在

Livox驱动会自动创建共享内存，但可以手动检查：

```bash
ls -l /home/$USER/timeshare
```

如果文件不存在，Livox驱动首次运行时会创建。

### 3. 启动Livox雷达驱动

```bash
roslaunch livox_ros_driver2 msg_MID360.launch
# 或根据你的雷达型号选择对应的launch文件
```

### 4. 启动相机驱动（带同步）

```bash
roslaunch take_picture_ros camera_with_sync.launch
```

### 5. 验证时间同步

检查话题时间戳是否一致：

```bash
# 终端1：查看雷达点云时间戳
rostopic echo /livox/lidar --noarr | grep stamp

# 终端2：查看相机图像时间戳
rostopic echo /camera/image_rgb --noarr | grep stamp
```

两者的时间戳应该非常接近（误差在毫秒级）。

## 配置参数说明

### launch文件参数

| 参数名        | 类型   | 默认值              | 说明              |
| ------------- | ------ | ------------------- | ----------------- |
| `topic_name`  | string | `/camera/image_rgb` | 图像发布话题      |
| `pipeline`    | string | (见launch文件)      | GStreamer管道配置 |
| `rate_hz`     | int    | 30                  | 图像发布频率(Hz)  |
| `enable_sync` | bool   | true                | 是否启用时间同步  |

### 修改pipeline配置

根据你的相机设备修改 `pipeline` 参数，例如：

**USB相机 (MJPEG):**

```xml
<param name="pipeline" value="v4l2src device=/dev/video0 ! image/jpeg, width=1920, height=1080, framerate=30/1 ! jpegdec ! videoconvert ! appsink sync=false drop=true max-buffers=1" />
```

**USB相机 (RAW):**

```xml
<param name="pipeline" value="v4l2src device=/dev/video0 ! video/x-raw, width=1920, height=1080, framerate=30/1 ! videoconvert ! appsink sync=false drop=true max-buffers=1" />
```

## 与MVS相机驱动的对比

| 特性         | mvs_ros_driver     | take_picture_ros (修改后) |
| ------------ | ------------------ | ------------------------- |
| 硬件触发支持 | ✓ (通过GPIO Line0) | ✗ (软件同步)              |
| 时间同步方式 | 共享内存           | 共享内存                  |
| 相机类型     | 海康威视工业相机   | USB/GStreamer支持的相机   |
| 触发延迟     | 更低（硬件触发）   | 稍高（软件同步）          |

**注意**：新方案使用软件同步，相比MVS的硬件触发会有额外延迟（通常<10ms）。如果需要高精度同步，建议：

1. 提高相机采集频率
2. 在后处理中进行时间戳插值对齐

## 多相机配置

如需使用多个相机，可以创建多个节点实例：

```xml
<launch>
    <!-- 相机1 -->
    <node pkg="take_picture_ros" type="camera_ros_driver_one_with_sync"
          name="camera1" output="screen">
        <param name="topic_name" value="/camera1/image_rgb" />
        <param name="pipeline" value="v4l2src device=/dev/video0 ! ..." />
        <param name="enable_sync" value="true" />
    </node>

    <!-- 相机2 -->
    <node pkg="take_picture_ros" type="camera_ros_driver_one_with_sync"
          name="camera2" output="screen">
        <param name="topic_name" value="/camera2/image_rgb" />
        <param name="pipeline" value="v4l2src device=/dev/video1 ! ..." />
        <param name="enable_sync" value="true" />
    </node>
</launch>
```

## 故障排查

### 1. 共享内存打开失败

**现象**：终端输出警告 "Failed to open shared memory file"

**解决方法**：

- 确认Livox驱动已经运行
- 检查文件权限：`ls -l /home/$USER/timeshare`
- 手动创建文件（仅用于测试）：
  ```bash
  dd if=/dev/zero of=/home/$USER/timeshare bs=16 count=1
  ```

### 2. 时间戳不同步

**现象**：雷达和相机的时间戳差异很大

**解决方法**：

- 检查 `enable_sync` 参数是否为 `true`
- 确认共享内存中有有效数据（`pointt->low != 0`）
- 重启Livox驱动和相机驱动

### 3. 相机无法打开

**现象**：错误 "Failed to open the camera"

**解决方法**：

- 检查设备路径：`ls /dev/video*`
- 测试GStreamer管道：
  ```bash
  gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! autovideosink
  ```
- 检查相机权限：`sudo chmod 666 /dev/video0`

## 调试技巧

### 启用DEBUG日志

在launch文件中添加：

```xml
<env name="ROSCONSOLE_CONFIG_FILE" value="$(find take_picture_ros)/config/debug.conf"/>
```

创建 `config/debug.conf`：

```
log4j.logger.ros.take_picture_ros=DEBUG
```

### 查看共享内存内容

```bash
# 以十六进制显示
hexdump -C /home/$USER/timeshare

# 或使用Python脚本
python3 << EOF
import struct
import mmap

with open('/home/$USER/timeshare', 'r+b') as f:
    mm = mmap.mmap(f.fileno(), 0)
    high, low = struct.unpack('qq', mm[:16])
    print(f"High: {high}, Low: {low}")
    print(f"Time (seconds): {low / 1e9}")
EOF
```

## 性能优化建议

1. **降低图像分辨率**：如果不需要高分辨率，降低分辨率可以减少延迟
2. **调整缓冲区大小**：`max-buffers=1` 确保获取最新帧
3. **提高相机帧率**：高帧率可以提高时间戳匹配精度
4. **使用MJPEG格式**：相比RAW格式，MJPEG带宽需求更低

## 参考资料

- [Livox ROS Driver 2 文档](https://github.com/Livox-SDK/livox_ros_driver2)
- [GStreamer Pipeline 语法](https://gstreamer.freedesktop.org/documentation/tools/gst-launch.html)
- Linux共享内存编程：`man mmap`, `man shm_open`
