# 双臂遥操作数据采集使用说明

## 文件说明

- `dual_collect.py`：数据采集主入口。
- `dual_teleop.py`：Flexiv TDK 主从臂遥操作薄封装。
- `dual_collect_utils.py`：相机、夹爪、目录创建和数据保存工具。
- `ref/`：原外骨骼遥操作采集参考代码。

## 基本用法

在机器人运行环境中执行：

```bash
sh collect/run_dual_collect.sh
```

常用参数可以直接在 `run_dual_collect.sh` 顶部修改。

也可以直接用命令行执行：

```bash
python collect/dual_collect.py \
  -1 <master_robot_sn> \
  -2 <slave_robot_sn> \
  --slave-gripper-id <slave_xense_id> \
  --save-root <save_root>
```

其中：

- `-1, --first-sn`：主臂序列号。
- `-2, --second-sn`：从臂序列号。
- `--slave-gripper-id`：从端 Xense 夹爪 ID。
- `--save-root`：数据保存根目录。

## 不采集夹爪

如果本次不需要初始化和采集夹爪：

```bash
python collect/dual_collect.py \
  -1 <master_robot_sn> \
  -2 <slave_robot_sn> \
  --save-root <save_root> \
  --use-gripper false
```

此时不会初始化 Xense，保存的夹爪宽度固定为 `0.0`。

## 常用可选参数

```bash
--fps 30
--session-name record_test
--network-interface 192.168.2.102
--gripper-eps 0.0001
--gripper-wait-time 0.1
--null-space-period 0.1
```

`--network-interface` 可以重复传入多个 LAN 网卡 IPv4 地址。

## 键盘控制

程序启动后：

- `r`：激活主从遥操作。
- `s`：暂停主从遥操作。
- `c`：开始记录一条新轨迹。
- `v`：结束当前轨迹记录。
- `q`：退出采集。

推荐流程：

```text
启动程序 -> r 启动遥操作 -> c 开始记录 -> v 结束记录
移动机械臂回到起点 -> c 记录下一条 -> v 结束下一条
s 暂停遥操作 -> q 退出程序
```

每次按 `c` 都会创建一个新的轨迹目录，记录相机、从臂 TCP、从臂关节角和从端夹爪宽度。

## 数据结构

每次运行会在 `save_root` 下创建一个 session 目录：

```text
record_YYYYmmdd_HHMMSS/
  cam_327322062498/
    color/
    depth/
  cam_319522062799/
    color/
    depth/
  tcps/
    tcp_00000.npy
  angles/
    angle_00000.npy
  metadata.json
```

保存格式：

- `tcps/tcp_*.npy`：`[x, y, z, qx, qy, qz, qw, gripper_width]`
- `angles/angle_*.npy`：`[q1, q2, q3, q4, q5, q6, q7, gripper_width]`

其中 TCP 数据记录的是从臂状态。

## 主端 Angler 编码器控制夹爪

主端使用 Angler 编码器控制装置，可以在 `run_dual_collect.sh` 中设置：

```bash
USE_GRIPPER="true"
ANGLER_ID="/dev/ttyUSB0"
ANGLER_INDEX="1"
ANGLER_BAUDRATE="1000000"
ANGLER_GAP="-1"
ANGLER_STRICT="true"
ANGLER_OPEN_ANGLE="51.68"
ANGLER_CLOSE_ANGLE="16.61"
SLAVE_OPEN_WIDTH="0.085"
SLAVE_CLOSE_WIDTH="0.0"
```

编码器角度会被线性映射为从端夹爪目标宽度：

```text
ANGLER_CLOSE_ANGLE -> SLAVE_CLOSE_WIDTH
ANGLER_OPEN_ANGLE  -> SLAVE_OPEN_WIDTH
```

从端仍然使用 Xense，采集保存的 `gripper_width` 仍然来自 `slave_gripper.read()`。

---

## 数据后处理：转换为 HDF5

采集完成后使用 `postprocess/convert_to_hdf5.py` 将原始 session 目录批量转换为 HDF5 格式。

### 输出 HDF5 Schema

每个 session 生成一个 `.hdf5` 文件，结构如下：

```text
actor/
  prism   (N, 7)  float32   主臂 TCP：[x, y, z, qx, qy, qz, qw]
  slot    (N, 7)  float32   从臂 TCP：[x, y, z, qx, qy, qz, qw]
atom/
  id      (N,)    int64     episode 编号（从 --episode-start 开始自增）
  tag     (N,)    |S5       帧标签，默认 b"move"
embodiment/
  ee      (N, 7)  float32   从臂末端位姿（同 actor/slot）
  joint   (N, 8)  float32   从臂关节角·7 + 夹爪宽度·1
observation/
  head/
    rgb   (N,)    |S{max}   头部相机彩色帧，JPEG 编码字节
step      (N,)    int64     帧序号 [0, 1, …, N-1]
tactile/
  left_gsmini/
    depth (N, H, W)  float32   左 Xense 深度图
    rgb   (N,)       |S{max}   左 Xense rectify 图，JPEG 编码字节
  right_gsmini/
    depth (N, H, W)  float32   右 Xense 深度图
    rgb   (N,)       |S{max}   右 Xense rectify 图，JPEG 编码字节
```

> 相机帧与 Xense 帧通过时间戳最近邻对齐到机器人时间轴。

### 转换命令

**转换单个 session：**

```bash
conda run -n collection python3 postprocess/convert_to_hdf5.py \
    --save-root /data/raw/record_20240101_120000_000000 \
    --out-dir   /data/hdf5
```

**批量转换（多进程并行）：**

```bash
conda run -n collection python3 postprocess/convert_to_hdf5.py \
    --save-root /data/raw \
    --out-dir   /data/hdf5 \
    --workers   8
```

**常用可选参数：**

```bash
--workers        8          # 并行进程数（默认 min(8, cpu_count)）
--jpeg-quality   95         # JPEG 质量 1–100（默认 95）
--encode-threads 4          # 每个进程内 JPEG 编码线程数（默认 4）
--cam-role       cam_327322062498=head   # 指定相机目录到角色的映射
--xense-role     xense_left=left_gsmini  # 指定 Xense 目录到角色的映射
--tag            move       # atom/tag 标签字符串（最多 5 字符）
--episode-start  0          # 起始 episode 编号
--overwrite                 # 覆盖已有 HDF5 文件（默认跳过）
```

**转换后验证 schema：**

```bash
conda run -n collection python3 postprocess/read_output_schema.py \
    --hdf5 /data/hdf5/record_20240101_120000_000000.hdf5
```

生成 `record_20240101_120000_000000_schema.json`，格式与 `data/hdf5_schema.json` 一致。
