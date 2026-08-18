# 机器人工程

- 这是一个实现数个任务的，通过上位机和单片机进行控制的机器人工程

- 这个工作空间用来存放单片机（STM32H723ZGT6）的代码

- 与上位机的 USART 协议、FreeRTOS 任务架构等**已定事项**见下文；改动时先读本文件保持同步

## 任务

机器人一共要完成三个任务

1. 从启动区移动到指定位置，该位置有一个三个方块（每个边长35cm, 650g左右）竖向堆叠的"塔"， 机器人要在不推倒塔的情况下把最上方的方块上下翻转180°

2. 完成任务1后要经过一个高低不一的四平台（高的距离地面40cm，低的距离地面20cm），有三个相同颜色（颜色已知）和一个不同颜色的方块，识别出需要取的三个颜色相同的方块，经过一个15°，水平1.5m的向上斜坡把方块运送到斜坡上的地面

3. 把运送后的方块堆叠成一个竖向放置的塔

## 机器人机构

1. **主控**

    - 迷你主机

    - STM32H723ZGT6

2. **底盘**

    - 麦克纳姆轮*4

    - DJI M3508\*4 + c620电调\*4(CAN控制，FDCAN1/2/3 已初始化于 CubeMX)

3. **机械臂**

    - 可伸缩升降的机械臂*2

    - M3508电机*4控制伸缩升降

    - GO8010电机（RS485协议）* 2进行机械臂末端关节的旋转

    - 其中一个机械臂有一个吸盘，另外一个机械臂有两个吸盘，都使用电磁阀控制

4. **视觉寻路**

    - 2D激光雷达

    - 双目摄像头（可以检测深度）

## 任务思路

1. **任务1**

主机给主控板发信号（USART 协议已定，见下），主控板收到信号后利用 M3508 编码器（已实现，经 macnum 做里程计与位置环）做里程计，运动到大概位置后，使用雷达进行校准到较为准确的位置，机械臂用吸盘吸附到方块后举起，然后关节电机旋转180°，再放下

2. **任务2**

之后机器人运动到高低不一的平台边，边运动边识别是否需要吸取，使用总共三个吸盘吸取方块后，一并上坡运动到平台上

3. **任务3**

到平台上后机械臂依次从下向上堆叠放置

## USART 通信协议（已定）

- 实现位于 `repos/chassis_repo/`：`interact_cmds.h`（命令/frame常量）、`uart_interact.c/h`（解析状态机 + 打包 + ACK）

- **帧格式**：`0x55 | CMD | LEN | DATA[0..LEN-1] | SUM | 0xBB`
  - LEN = DATA 字节数（不含头/CMD/LEN/SUM/帧尾）
  - SUM = 从 0x55 到 DATA 末尾逐字节异或（不含 0xBB）
  - 数据统一小端；有 LEN 所以数据内出现 0x55/0xBB 无需转义

- **命令字**（下行 0x00~0x7F，上行 0x80~0xFF；注意 CMD_HEARTBEAT=0xA0 是例外，按内容路由）：

  | 方向 | CMD | 数据域 | 含义 |
  |------|-----|--------|------|
  | 下行 | 0x01 | f32 | 目标停车距离【回执】|
  | 下行 | 0x02 | u8 valid + f32 center + f32 slope | 巡线数据【高频不回】|
  | 下行 | 0x03 | u8 valid + f32 dist | 到塔距离【高频不回】|
  | 下行 | 0x04 | u8 flag | T路口/右转信号【回执】|
  | 下行 | 0x05 | u8 | 任务控制 0复位/1启动/2暂停【回执】|
  | 下行 | 0x10 | f32 vx+vy+omega | 手动调速度(调试)【不回】|
  | 下行 | 0xA0 | u8 seq | 上位机心跳，MCU 回执 |
  | 上行 | 0x81 | 空 | 第一次右转完成 |
  | 上行 | 0x82 | 空 | 第二次右转完成 |
  | 上行 | 0x83 | 空 | 任务完成 |
  | 上行 | 0x84 | f32 x+y+yaw | 状态回传(可选) |
  | 上行 | 0x85 | u8 cmd + u8 result | EVT_ACK 命令应答(回执) |

- **ACK/回执机制**：单次命令（0x01/0x04/0x05/0xA0）处理成功后由 `UartInteract_RequestAck()` 挂起，下一拍 `UartInteract_Poll()` 发 `55 85 02 {ack_cmd}{result} SUM BB`；长度错误回 result=1(NACK)。缓冲发送而非在接收回调里直接发，避免中断里阻塞。

- **单位约定**：底盘/里程计全部 **m / rad**（`rea_x/y` m，yaw rad，速度 m/s，omega rad/s）；`target_distance` 与 `tower_distance` 必须同一物理单位（建议统一 m，默认 0.3 即 0.3m）；`line_center/slope` 是**图像域**（非米），巡线 PID 输入需自行换算/归一化。

## FreeRTOS 任务架构（已定）

- 实现位于 `Core/Src/freertos.c`（CubeMX 生成的模板，改动只写在 USER CODE 区）

- **2 个任务 + 1 个字节队列**：

  | 任务 | 优先级 | 栈(字) | 职责 |
  |------|--------|--------|------|
  | ChassisMainTask | osPriorityRealtime | 256 | 1kHz 控制：`osThreadFlagsWait(0x01)` 等 TIM1 信号 → `Chassis_Task1_Update(&t1)`（内部含 `Chassis_Update`）|
  | Comms_Task | osPriorityLow | 256 | 串口通信：`osMessageQueueGet(CmdQueueTask1,..,10ms)` 阻塞 → `UartInteract_RxByte` 喂解析 → `Poll` 发事件/ACK |

- **节奏机制**：TIM1 = 1kHz（PSC=274, Period=999）。TIM1 中断里 `osThreadFlagsSet(ChassisMainTaskHandle,0x01)`（带 NULL 保护）唤醒控制任务；控制任务用 `osThreadFlagsWait` **阻塞等待**，保证让出 CPU 不饿死 Comms。Comms 靠队列阻塞，天然让出。

- **串口收**：USART3 @115200，中断单字节。`main.c` 初始化装填一次 `HAL_UART_Receive_IT(&huart3,(uint8_t*)&rx_byte,1)`，`HAL_UART_RxCpltCallback`（==USART3）里 `osMessageQueuePut` 入队 + 重新装填。`rx_byte` 为 `volatile`。

- **队列**：`CmdQueueTask1`，32 × uint8_t（串口字节队列，FIFO）。

- **串口发**：`uart_tx_hook`（main.c）用 `HAL_UART_Transmit` 阻塞发送，只在 Comms 任务上下文调用。

- **尚待补充**：
  - 手动接管分支：`CMD_SET_VELOCITY` 已置 `it.task_paused=1`，但控制任务目前**没检查 `UartInteract_IsPaused()`**（约定本阶段先不加）——后续加回去避免手控与自动状态机打架
  - 互斥量：`Chassis_Task1` 输入字段由 Comms 写、控制读，目前未加锁（vision<100Hz 时风险低，但 double 非原子）；需要时再加
  - 监控/看门狗/上位机断联急停任务：尚未实现
  - 机械臂（M3508 伸缩、GO8010/DrEmpower 关节）、吸盘电磁阀的任务划分：尚未定

## 构建与烧录

- 构建：`cmake -S . -B build/Debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake`，然后 `cmake --build build/Debug`
  - **注意**：顶层 `CMakeLists.txt` 里 `"-lm"` 必须以 `target_link_libraries` 的**最后一个链接项**加入（放 chassis 之后），否则 `chassis.c/macnum.c` 的 `cos/sin` 无法解析
- 烧录（需先 `sudo apt install stlink-tools openocd gdb-multiarch`，并确保 udev 权限）：
  - `arm-none-eabi-objcopy -O binary main.elf main.bin`
  - `st-flash --reset write main.bin 0x08000000`
  - 调试可选用 openocd/st-util + gdb-multiarch

## 代码层面结构（分层）

- 依赖方向单向向下：`main.c/freertos.c`(App) → `chassis_repo`(chassis+chassis_task_1+uart_interact+interact_cmds) → `m3508_repo` + `macnum_repo` → HAL(USD)
- FreeRTOS 直接调用的只有 `freertos.c` 两个任务入口；业务被层层调用（控制链：任务→Chassis_Task1→Chassis→Macnum/M3508→CAN；通信链：任务→UartInteract→Chassis_Task1/Chassis+发送钩子→USART）
- 引入HAL代码时使用`#include "stm32h7xx_hal.h"`
