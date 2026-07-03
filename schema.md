# 数据保存 Schema

## 目录结构

```
{SAVE_ROOT}/
└── {SESSION_NAME}_{YYYYMMDD_HHMMSS_ffffff}/
    ├── metadata.json
    ├── timestamps/
    │   ├── robot_timestamps.npy
    │   ├── cam_{cam_name}_timestamps.npy
    │   ├── xense_xense_left_timestamps.npy       # 仅当 XENSE_LEFT 启用
    │   └── xense_xense_right_timestamps.npy      # 仅当 XENSE_RIGHT 启用
    ├── tcps/
    │   └── tcp_{NNNNN}.npy
    ├── angles/
    │   └── angle_{NNNNN}.npy
    ├── master_tcps/
    │   └── tcp_{NNNNN}.npy
    ├── master_angles/
    │   └── angle_{NNNNN}.npy
    ├── {cam_name}/                               # e.g. cam_327322062498
    │   ├── color/
    │   │   └── {NNNNNNNNNNNNNNNN}.png
    │   └── depth/
    │       └── {NNNNNNNNNNNNNNNN}.png
    ├── xense_left/                               # 仅当 XENSE_LEFT 启用
    │   ├── rectify/
    │   │   └── {NNNNN}.npy
    │   └── depth/
    │       └── {NNNNN}.npy
    └── xense_right/                              # 仅当 XENSE_RIGHT 启用
        ├── rectify/
        │   └── {NNNNN}.npy
        └── depth/
            └── {NNNNN}.npy
```

---

## metadata.json

记录本次采集的所有配置参数，JSON 格式，采集开始时写入一次。

| 字段 | 类型 | 说明 |
|---|---|---|
| `first_sn` | str | Master 机器人序列号 |
| `second_sn` | str | Slave 机器人序列号 |
| `fps` | int | 机器人 / RGB-D 采样率 Hz |
| `xense_fps` | int | 视触觉传感器采样率 Hz |
| `use_gripper` | bool | 是否启用夹爪同步采集 |
| `slave_gripper_id` | str | Slave Xense 夹爪设备 ID |
| `xense_left_id` | str \| null | 左视触觉传感器设备 ID，未启用时为 null |
| `xense_right_id` | str \| null | 右视触觉传感器设备 ID，未启用时为 null |
| `recorded_robot` | str | 固定为 `"second"`（slave arm） |
| `tcp_pose_source` | str | TCP 位姿来源描述 |
| `tdk_tcp_pose_order` | str | `"[x, y, z, qw, qx, qy, qz]"` |
| `saved_tcp_pose_order` | str | `"[x, y, z, qx, qy, qz, qw]"` |
| `master_gripper_width_source` | str | Master 夹爪宽度来源描述 |
| `slave_gripper_width_source` | str | Slave 夹爪宽度来源描述 |
| `camera_serials` | dict | `{cam_name: serial_str}` |
| `created_at` | str | ISO 8601 绝对时间戳 |

---

## 机器人状态

**帧号格式**: `{NNNNN}` — 5 位十进制，从 `00000` 起连续递增。
**采样率**: `FPS` Hz（默认 30 Hz），rate-control 控制。
**时间戳**: `timestamps/robot_timestamps.npy`，时钟为 `time.perf_counter()`（秒）。

| 目录 | 文件名 | dtype | shape | 字段顺序 |
|---|---|---|---|---|
| `tcps/` | `tcp_{N}.npy` | float64 | `(8,)` | `x, y, z, qx, qy, qz, qw, slave_gripper_width` |
| `angles/` | `angle_{N}.npy` | float64 | `(8,)` | `q1, q2, q3, q4, q5, q6, q7, slave_gripper_width` |
| `master_tcps/` | `tcp_{N}.npy` | float64 | `(8,)` | `x, y, z, qx, qy, qz, qw, slave_gripper_width` |
| `master_angles/` | `angle_{N}.npy` | float64 | `(8,)` | `q1, q2, q3, q4, q5, q6, q7, slave_gripper_width` |

单位：位置 m，关节角 rad，夹爪宽度 m。
四元数顺序为 `[qx, qy, qz, qw]`（已从 TDK 原始顺序 `[qw, qx, qy, qz]` 转换）。
`master_tcps` 和 `master_angles` 中的 `slave_gripper_width` 字段与 `tcps` / `angles` 中相同（均来自 slave 夹爪读数），便于训练时统一索引。

---

## RGB-D 相机

**帧号格式**: `{NNNNNNNNNNNNNNNN}` — 16 位十进制，从 `0000000000000000` 起连续递增。
**采样率**: `FPS` Hz，由 `pipeline.wait_for_frames()` 硬件节拍驱动，无 sleep。
**时间戳**: `timestamps/cam_{cam_name}_timestamps.npy`，时钟为 `time.perf_counter()`（秒）。

| 子目录 | 格式 | 分辨率 | 像素格式 | 说明 |
|---|---|---|---|---|
| `color/` | PNG | 640 × 480 | BGR8 | 彩色图像 |
| `depth/` | PNG | 640 × 480 | uint16 (Z16) | 深度图，单位 mm |

同一帧号的 color 与 depth 图来自同一次 `wait_for_frames()` 调用，硬件对齐。

---

## 视触觉传感器

**帧号格式**: `{NNNNN}` — 5 位十进制，从 `00000` 起连续递增。
**采样率**: `XENSE_FPS` Hz（默认 50 Hz），rate-control 控制。
**时间戳**: `timestamps/xense_xense_left_timestamps.npy` / `timestamps/xense_xense_right_timestamps.npy`，时钟为 `time.perf_counter()`（秒）。

每帧通过一次 `sensor.selectSensorInfo(Rectify, Depth)` 调用获取两路输出，保证同帧数据时序一致。

| 子目录 | 文件名 | dtype | shape | 内容 |
|---|---|---|---|---|
| `rectify/` | `{N}.npy` | 由 SDK 决定 | 由传感器配置决定 | 校正后的原始触觉图像阵列 |
| `depth/` | `{N}.npy` | 由 SDK 决定 | 由传感器配置决定 | 接触面深度图 |

`dtype` 和具体 `shape` 在运行时由 `sensor.selectSensorInfo()` 返回值确定，各传感器型号可能不同，后处理时以实际数组为准。

---

## 时间戳

所有流的时间戳均采用 **同一时钟** `time.perf_counter()`（单调递增，秒，无绝对零点），可直接跨流做最近邻对齐。时间戳记录点为每次数据读取返回后立即采样。

| 文件 | dtype | shape | 对应数据流 |
|---|---|---|---|
| `robot_timestamps.npy` | float64 | `(N_robot,)` | `tcps` / `angles` / `master_tcps` / `master_angles` 逐帧时间戳 |
| `cam_{cam_name}_timestamps.npy` | float64 | `(N_cam,)` | 对应相机 color + depth 帧 |
| `xense_xense_left_timestamps.npy` | float64 | `(N_xense_left,)` | `xense_left/rectify` 帧 |
| `xense_xense_right_timestamps.npy` | float64 | `(N_xense_right,)` | `xense_right/rectify` 帧 |

各流帧数 `N_robot`、`N_cam`、`N_xense_*` **不要求相等**。各流独立采集，互不阻塞，后处理按时间戳最近邻匹配对齐。
