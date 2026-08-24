# kfs_vision

`kfs_vision` 是 KFS RGB-D 视觉节点。节点直接使用 Orbbec C++ SDK 获取同步 RGB-D，保持原程序的处理顺序：

```text
Orbbec FrameSet
  -> 深度对齐到彩色图
  -> 对齐后的深度补洞
  -> YOLO26-seg CUDA 推理
  -> 深度门控和前平面拟合
  -> 水平位姿 x/y/yaw
  -> 四个字段全部有效时发布 ROS 消息
```

节点不订阅 Orbbec ROS 图像 topic。运行本节点时，不要同时启动 `orbbec_camera`，否则两个进程会争用同一台相机。

## 1. ROS 接口

### 1.1 发布话题

默认话题：

```text
/kfs/target
```

消息类型：

```text
custom_msgs/msg/KfsTarget
```

消息内容：

```msg
uint8 BLUE=0
uint8 RED=1

uint8 color
float32 x_m
float32 y_m
float32 yaw_rad
```

字段含义：

| 字段 | 含义 |
| --- | --- |
| `color` | `0=BLUE`，`1=RED` |
| `x_m` | 相对相机左右位置，右为正，单位 m |
| `y_m` | 相对相机前向距离，单位 m |
| `yaw_rad` | 水平偏航角，单位 rad，正负方向与原程序一致 |

### 1.2 发布条件

以下条件全部满足时才发布：

1. YOLO 检测到目标；
2. 类别为蓝色或红色；
3. 深度门控通过；
4. 前平面拟合成功；
5. 水平位姿计算成功；
6. `x_m`、`y_m`、`yaw_rad` 都是有限数值。

任一条件失败都不会发布“无效消息”，只在终端显示原因。订阅端必须设置超时，例如 300 ms 没有收到新消息就停止使用旧目标。

QoS 为：

```text
reliable + volatile + keep_last(1)
```

## 2. 系统要求

- Ubuntu 24.04 x86_64；
- ROS 2 Jazzy；
- NVIDIA 驱动；
- CUDA 13；
- 与 ONNX Runtime 1.29.0 CUDA 13 包兼容的 cuDNN；
- OrbbecSDK 2.9.3 开发文件和 udev 规则；
- OpenCV 4；
- PCL 1.14；
- Eigen3；
- nlohmann-json；
- 已存在并可构建的 `custom_msgs` 包。

快速检查：

```bash
source /opt/ros/jazzy/setup.bash
nvidia-smi
readlink -f /usr/local/lib/libOrbbecSDK.so.2
```

预期 Orbbec 运行库版本为 `libOrbbecSDK.so.2.9.3`。如果安装在其他前缀，构建时通过 `-DOrbbecSDK_DIR=...` 指定 CMake 配置目录，不修改源码。

## 3. 准备 ONNX Runtime

ONNX Runtime CUDA SDK 体积较大，不提交到普通 Git。首次构建前，在仓库根目录执行：

```bash
cd ros2_ws
./src/kfs_vision/scripts/setup_onnxruntime.sh
```

脚本会下载固定的官方包：

```text
onnxruntime-linux-x64-gpu_cuda13-1.29.0.tgz
```

并验证 SHA-256：

```text
844c64acfc43ab9423215c26493055ea229268e28283146cc644ecef0bdae048
```

如果电脑不能联网，可以把已解压且完整的 SDK 复制到本机，然后执行：

```bash
./src/kfs_vision/scripts/setup_onnxruntime.sh \
  --from /absolute/path/to/onnxruntime-linux-x64-gpu_cuda13-1.29.0
```

`--from` 必须是绝对路径。脚本不会覆盖已有的不完整目录；遇到不完整目录时会报错并要求人工移走，避免误删文件。

## 4. 构建

从仓库根目录进入工作空间：

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
```

OrbbecSDK 2.9.3 官方安装器在本机把 CMake 配置直接放在 `/usr/local/lib`。这种布局下，开发机推荐：

```bash
colcon build --packages-up-to kfs_vision --symlink-install \
  --cmake-args \
  -DOrbbecSDK_DIR=/usr/local/lib
source install/setup.bash
```

正式部署：

```bash
colcon build --packages-up-to kfs_vision \
  --cmake-args \
  -DOrbbecSDK_DIR=/usr/local/lib
source install/setup.bash
```

如果 OrbbecSDK 不在 CMake 默认搜索路径：

```bash
colcon build --packages-up-to kfs_vision \
  --cmake-args \
  -DOrbbecSDK_DIR=/absolute/path/to/orbbec/cmake
```

不要把 `OrbbecSDK_DIR` 写进源码或提交包含本机路径的 CMake 缓存。

## 5. 构建后检查

运行单元测试：

```bash
colcon test --packages-select kfs_vision
colcon test-result --verbose
```

测试覆盖：

- 原几何与位姿计算；
- 蓝色和红色映射；
- mm 到 m；
- deg 到 rad；
- 类别、平面、位姿缺失时不生成消息；
- `NaN` 或 `Inf` 时不生成消息。

检查安装后的动态库：

```bash
ldd install/kfs_vision/lib/kfs_vision/kfs_vision_node
readelf -d install/kfs_vision/lib/kfs_vision/kfs_vision_node | \
  grep -E 'RPATH|RUNPATH'
```

验收要求：

- 没有 `not found`；
- ONNX Runtime 来自 `install/kfs_vision/lib/`；
- OrbbecSDK 来自 `install/kfs_vision/lib/libOrbbecSDK.so.2`；
- `install/kfs_vision/lib/extensions/` 中包含 Orbbec 的
  `frameprocessor`、`filters`、`depthengine` 和 `firmwareupdater` 扩展；
- 安装节点的 RPATH 为 `$ORIGIN/..`；
- 不从其他源码工作空间加载 ONNX Runtime 或 OrbbecSDK。

## 6. 无相机模型检查

在打开相机之前，可以先验证 ONNX Runtime、CUDA provider 和模型：

```bash
source install/setup.bash
ros2 run kfs_vision kfs_model_check
```

成功输出应包含：

```text
Inference provider: CUDAExecutionProvider
CUDA ONNX model check passed (session creation + one inference).
```

检查另一个模型：

```bash
ros2 run kfs_vision kfs_model_check \
  --model /absolute/path/to/model.onnx
```

## 7. 推荐启动方式：ros2 launch

默认无 GUI 启动：

```bash
source install/setup.bash
ros2 launch kfs_vision kfs_vision.launch.py
```

launch 会自动从安装空间查找：

```text
share/kfs_vision/models/exp.onnx
share/kfs_vision/config/kfs_plane_fit.json
share/kfs_vision/config/kfs_vision.yaml
```

因此启动前不需要进入包源码目录，也不需要手动设置模型路径或 `LD_LIBRARY_PATH`。

打开 OpenCV 调试窗口：

```bash
ros2 launch kfs_vision kfs_vision.launch.py show_gui:=true
```

使用其他模型：

```bash
ros2 launch kfs_vision kfs_vision.launch.py \
  model_path:=/absolute/path/to/model.onnx
```

使用其他平面参数 JSON：

```bash
ros2 launch kfs_vision kfs_vision.launch.py \
  plane_config_path:=/absolute/path/to/kfs_plane_fit.json
```

保存 GUI 滑块参数到指定文件：

```bash
ros2 launch kfs_vision kfs_vision.launch.py \
  show_gui:=true \
  plane_config_output_path:=/absolute/writable/path/kfs_plane_fit.json
```

如果没有设置 `plane_config_output_path`，GUI 滑块只影响当前运行，不会修改安装目录中的默认 JSON。

使用其他 YAML：

```bash
ros2 launch kfs_vision kfs_vision.launch.py \
  params_file:=/absolute/path/to/kfs_vision.yaml
```

提高日志等级：

```bash
ros2 launch kfs_vision kfs_vision.launch.py log_level:=debug
```

使用命名空间：

```bash
ros2 launch kfs_vision kfs_vision.launch.py namespace:=robot1
```

此时节点和默认话题变为：

```text
/robot1/kfs_vision_node
/robot1/kfs/target
```

## 8. 直接 ros2 run

节点内部有完整默认参数，因此也可以直接运行：

```bash
source install/setup.bash
ros2 run kfs_vision kfs_vision_node
```

打开 GUI：

```bash
ros2 run kfs_vision kfs_vision_node \
  --ros-args \
  -p show_gui:=true
```

修改置信度：

```bash
ros2 run kfs_vision kfs_vision_node \
  --ros-args \
  -p yolo_confidence:=0.60
```

覆盖模型和 JSON：

```bash
ros2 run kfs_vision kfs_vision_node \
  --ros-args \
  -p model_path:=/absolute/path/to/model.onnx \
  -p plane_config_path:=/absolute/path/to/kfs_plane_fit.json
```

加载 YAML：

```bash
ros2 run kfs_vision kfs_vision_node \
  --ros-args \
  --params-file /absolute/path/to/kfs_vision.yaml
```

覆盖的模型、JSON 和输出路径必须是绝对路径。默认包内路径由节点自动查询，不受工作空间位置和用户名影响。

## 9. 参数表

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `model_path` | string | 空 | 空时使用包内 `models/exp.onnx` |
| `plane_config_path` | string | 空 | 空时使用包内 `kfs_plane_fit.json` |
| `plane_config_output_path` | string | 空 | GUI 滑块可选保存位置 |
| `target_topic` | string | `kfs/target` | 目标消息相对话题名 |
| `frame_width` | int | `1280` | 彩色流宽度 |
| `frame_height` | int | `720` | 彩色流高度 |
| `camera_fps` | int | `30` | 请求彩色流帧率 |
| `yolo_confidence` | double | `0.50` | YOLO 置信度门限，范围 `[0,1]` |
| `front_housing_from_depth_zero_mm` | double | `4.930` | 相机参考面偏移，单位 mm |
| `show_gui` | bool | `false` | 是否显示 OpenCV 窗口 |
| `terminal_period_ms` | int | `1000` | 终端状态汇总周期 |

第一版参数在节点启动时读取。修改模型、分辨率或相机参数后应重启节点。

查看实际参数：

```bash
ros2 param list /kfs_vision_node
ros2 param dump /kfs_vision_node
```

## 10. 查看消息和节点

查看接口：

```bash
ros2 interface show custom_msgs/msg/KfsTarget
```

查看节点：

```bash
ros2 node info /kfs_vision_node
```

查看话题：

```bash
ros2 topic info /kfs/target --verbose
ros2 topic echo /kfs/target
ros2 topic hz /kfs/target
```

如果当前目标无效，`echo` 和 `hz` 没有新输出是预期行为，应同时观察 `kfs_vision` 终端中的失败原因。

## 11. 终端信息

节点默认每秒输出一条汇总。

节点启动 OrbbecSDK 前会关闭 SDK 自己的文件日志，因此不会在启动目录生成
`Log/OrbbecSDK.log.txt`。KFS 状态仍通过 ROS 日志显示在当前终端；ROS 2
自身的运行日志仍按标准行为保存在 `~/.ros/log/`。

有效结果示例：

```text
process_fps=29.2 target=valid color=RED x=0.124m y=0.681m yaw=-0.084rad ...
```

无检测示例：

```text
process_fps=29.1 target=none reason=no detection ...
```

测量失败示例：

```text
process_fps=29.0 target=invalid reason=horizontal border clipped ...
```

终端还显示置信度、mask、边界框、采样数、平面 RMS、内点数和各阶段耗时。这些数据不会放入 ROS 消息。

## 12. GUI 操作

使用 `show_gui:=true` 后：

- `q`：请求 ROS 节点正常退出；
- 调整滑块：立即影响当前平面拟合参数；
- 未设置输出路径：只在内存中生效；
- 设置 `plane_config_output_path`：滑块变化自动保存到该文件。

部署为无桌面服务时必须保持 `show_gui=false`。

## 13. 正常停止

终端中按：

```text
Ctrl+C
```

节点最多可能等待当前 `waitForFrames(1000)` 返回，然后停止相机 pipeline、回收视觉线程并退出。GUI 模式也可以按 `q`。

相机故障采用“失败即退出”，不在节点内部等待重连：

- SDK 报告目标相机已从设备列表移除：退出；
- 相机仍可枚举，但连续 3 次、每次 1 秒都没有取得帧：退出；
- 对齐结果缺失，或对齐帧缺少彩色/深度数据：退出；
- 上述运行时故障返回退出码 `1`，`ros2 launch` 默认不会自动重启节点；
- 正常 Ctrl+C 或 GUI 中按 `q` 返回退出码 `0`。

相机断联退出后，重新连接相机并重新执行 launch。目标暂时未被检测到、深度门控失败或位姿无效只代表当前帧不发布消息，不属于相机断联，节点会继续处理后续帧。

检查是否残留：

```bash
ros2 node list | grep kfs
pgrep -af kfs_vision_node
```

## 14. 常见问题

### 14.1 `ONNX Runtime CUDA 13 SDK is missing or incomplete`

先执行：

```bash
./src/kfs_vision/scripts/setup_onnxruntime.sh
```

然后重新构建。

### 14.2 `CUDAExecutionProvider is unavailable`

依次检查：

```bash
nvidia-smi
ldd install/kfs_vision/lib/libonnxruntime_providers_cuda.so | \
  grep 'not found'
```

重点核对 NVIDIA 驱动、CUDA 13 和 cuDNN 版本。

### 14.3 `no Orbbec camera detected`

- 确认 USB 连接和供电；
- 确认 Orbbec udev 规则已安装；
- 确认当前用户有设备权限；
- 确认没有同时运行 `orbbec_camera` 或其他占用相机的程序。

相机在运行期间断开时，节点输出 `Orbbec camera disconnected` 或对应的
OrbbecSDK 错误，然后以退出码 `1` 结束。节点不会原地等待相机重新插入；恢复连接后需要重新 launch。

如果报错 `Invalid filter name ... HoleFillingFilter`，或 Orbbec 日志提示
`extensions/frameprocessor/libob_frame_processor.so` 不存在，说明使用了只包含
`libOrbbecSDK.so`、但缺少运行时扩展的旧安装结果。清理并重新构建本包：

```bash
rm -rf build/kfs_vision install/kfs_vision
colcon build --packages-up-to kfs_vision \
  --cmake-args \
  -DOrbbecSDK_DIR=/usr/local/lib
source install/setup.bash
```

### 14.4 `aligned depth size does not match the color frame`

节点要求深度已通过 SDK 对齐到彩色分辨率。该错误表示相机 profile 或 SDK 行为与基准环境不一致，应检查 OrbbecSDK 版本和启动时打印的流信息，不应直接跳过检查。

### 14.5 没有 `/kfs/target` 消息

先看节点终端：

- `target=none`：没有 YOLO 检测；
- `depth rejected`：深度门控失败；
- `front plane fit failed`：平面拟合失败；
- `horizontal border clipped`：目标碰到水平图像边界；
- `visible width ... incomplete`：可见宽度不完整；
- `center is occluded`：中心带被遮挡。

这些情况按照消息契约都不会发布。

### 14.6 启动后加载了错误动态库

执行：

```bash
ldd install/kfs_vision/lib/kfs_vision/kfs_vision_node | \
  grep -E 'onnxruntime|Orbbec'
readelf -d install/kfs_vision/lib/kfs_vision/kfs_vision_node | \
  grep -E 'RPATH|RUNPATH'
```

节点安装时会把构建所选的 Orbbec runtime 和 ONNX Runtime 放到包的 `lib` 目录，并使用 `$ORIGIN/..`。若结果仍指向其他工作空间，不要用额外 `LD_LIBRARY_PATH` 掩盖，应先修正安装或重新构建。

## 15. 机器人部署流程

机器人首次部署：

1. 安装 ROS 2 Jazzy、OrbbecSDK 2.9.3、NVIDIA 驱动、CUDA 13 和 cuDNN；
2. 拉取包含 `custom_msgs` 和 `kfs_vision` 的仓库；
3. 准备 ONNX Runtime；
4. 清洁构建；
5. 运行测试和模型检查；
6. 连接相机后启动节点；
7. 验证 `/kfs/target` 和下游超时保护。

命令：

```bash
git pull
cd ros2_ws
source /opt/ros/jazzy/setup.bash

./src/kfs_vision/scripts/setup_onnxruntime.sh

rm -rf build/kfs_vision install/kfs_vision
colcon build --packages-up-to kfs_vision \
  --cmake-args \
  -DOrbbecSDK_DIR=/usr/local/lib
source install/setup.bash

colcon test --packages-select kfs_vision
colcon test-result --verbose
ros2 run kfs_vision kfs_model_check
ros2 launch kfs_vision kfs_vision.launch.py
```

这里的清理范围只包含 `kfs_vision` 的构建和安装目录，不删除整个工作空间。

每次模型、ONNX Runtime、OrbbecSDK 或 CUDA 环境变化后，都重新执行模型检查和真实相机测试。

## 16. 验收边界

以下结果代表不同层级，不能混为一谈：

- `colcon build`：只证明能够编译链接；
- 单元测试：证明几何逻辑和消息门控满足测试输入；
- `kfs_model_check`：证明当前机器能加载模型并执行一次 CUDA 推理；
- 节点启动：证明相机和处理循环能运行；
- `/kfs/target`：证明当前目标通过全部门控；
- 机器人现场验收：还必须覆盖蓝/红目标、遮挡、碰边、距离变化、目标丢失和控制超时。

本机测试通过后再提交并让机器人 `pull`。提交前确认 `custom_msgs`、`kfs_vision` 和模型文件都已被 Git 跟踪；`third_party` 按设计不会被 Git 跟踪，机器人需运行准备脚本。
