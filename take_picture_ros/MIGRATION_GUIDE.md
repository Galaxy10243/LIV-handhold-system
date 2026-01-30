# 时间同步切换对比

## 原始配置（MVS相机）

```
┌─────────────────────┐
│ Livox ROS Driver 2  │
│  (雷达驱动)          │
└──────────┬──────────┘
           │ 写入时间戳
           ↓
    ┌─────────────┐
    │ 共享内存     │
    │ /timeshare  │
    └──────┬──────┘
           │ 读取时间戳
           ↓
┌──────────────────────┐
│  mvs_ros_driver      │
│  (海康工业相机)       │
│  - 硬件触发(GPIO)    │
│  - 共享内存同步      │
└──────────────────────┘
```

## 新配置（Take Picture相机）

```
┌─────────────────────┐
│ Livox ROS Driver 2  │
│  (雷达驱动)          │
└──────────┬──────────┘
           │ 写入时间戳
           ↓
    ┌─────────────┐
    │ 共享内存     │
    │ /timeshare  │
    └──────┬──────┘
           │ 读取时间戳
           ↓
┌───────────────────────────────┐
│  take_picture_ros             │
│  camera_ros_driver_one_       │
│  with_sync                    │
│  (USB/GStreamer相机)          │
│  - 软件同步                   │
│  - 共享内存同步                │
└───────────────────────────────┘
```

## 关键变化

| 项目       | 原配置                  | 新配置                            |
| ---------- | ----------------------- | --------------------------------- |
| 相机驱动包 | `mvs_ros_driver`        | `take_picture_ros`                |
| 可执行文件 | `grabImgWithTrigger`    | `camera_ros_driver_one_with_sync` |
| 相机类型   | 海康威视工业相机        | USB/V4L2相机                      |
| 触发方式   | 硬件触发(GPIO Line0)    | 软件采集                          |
| 同步方式   | 共享内存                | 共享内存（相同）                  |
| 时间戳来源 | `/home/$USER/timeshare` | `/home/$USER/timeshare`（相同）   |
| 配置文件   | `.yaml`                 | launch参数                        |

## 启动命令对比

### 原配置启动

```bash
# 启动雷达
roslaunch livox_ros_driver2 msg_MID360.launch

# 启动MVS相机
roslaunch mvs_ros_driver mvs_camera_trigger.launch
```

### 新配置启动

```bash
# 启动雷达（相同）
roslaunch livox_ros_driver2 msg_MID360.launch

# 启动新相机驱动
roslaunch take_picture_ros camera_with_sync.launch
```

## 代码变化要点

### 1. 添加共享内存读取

```cpp
// 打开共享内存
const char *user_name = getlogin();
std::string path = "/home/" + std::string(user_name) + "/timeshare";
int fd = open(path.c_str(), O_RDWR);
time_stamp *pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp),
                                        PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
```

### 2. 使用同步时间戳

```cpp
ros::Time captureTime;
if (enable_sync && pointt != MAP_FAILED && pointt->low != 0) {
    // 使用Livox同步时间
    int64_t b = pointt->low;
    double time_pc = b / 1000000000.0;
    captureTime = ros::Time(time_pc);
} else {
    // 回退到系统时间
    captureTime = ros::Time::now();
}
```

### 3. 资源清理

```cpp
// 程序退出时
if (pointt && pointt != MAP_FAILED) {
    munmap(pointt, sizeof(time_stamp));
}
if (fd >= 0) {
    close(fd);
}
```
