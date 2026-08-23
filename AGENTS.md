# AGENTS.md — 20.standard_robot 项目学习记录

## 项目概况

DJI RoboMaster 步兵机器人标准代码，基于 STM32F407IGHx + FreeRTOS + HAL 库。

---

## 1. 硬件平台

| 项目 | 详情 |
|------|------|
| MCU | STM32F407IGHx (Cortex-M4F, 带 FPU) |
| 主频 | 168MHz (HSE 12MHz → PLL × 14 / 2) |
| RTOS | FreeRTOS V10.0.1, Tick = 1kHz |
| 编译工具 | Keil MDK (ARMCC) |
| IMU | BMI088 (陀螺仪+加速度计) + IST8310 (磁力计) |

---

## 2. CAN 总线架构（核心）

### 两条 CAN 总线

| 总线 | 外设 | 用途 |
|------|------|------|
| **CAN1** (GIMBAL_CAN) | PD0/PD1 | 云台电机 + 射击电机 + 摩擦轮 |
| **CAN2** (CHASSIS_CAN) | PB5/PB6 | 底盘电机 + Yaw 云台电机（跨总线） |

> ⚠️ Yaw(GM6020) 的**控制命令**在 CHASSIS_CAN 上，但**反馈接收**也在 CHASSIS_CAN (0x209)。Pitch(GM6020) 的控制在 GIMBAL_CAN 上，反馈通过 0x20A。

### DJI 电机 CAN 协议

**控制帧（发向电机）：**
- `0x200` — 底盘 3508 电机 1-4（每个电机 2 字节，共 8 字节）
- `0x1FF` — GIMBAL CAN 上的电机 5-8（摩擦轮等 C620 电调）
- `0x2FF` — GM6020 云台电机（电压控制模式，范围 ±30000）

**反馈帧（电机发回）：**
- 反馈 ID = `0x200 + 电机 ESC ID`
- 每个电机 8 字节：`ecd(2B) + speed_rpm(2B) + given_current(2B) + temperature(1B) + 保留(1B)`

### 本车电机 ID 分配 (GIMBAL_CAN)

| ESC ID | 反馈 ID | 电机 | 类型 | 用途 |
|--------|---------|------|------|------|
| 6 | 0x206 | Fric1 | C620+3508 | 摩擦轮 1 |
| 7 | 0x207 | Fric2 + Trigger | C620+3508 / 2006 | ⚠️ 冲突！ |
| 7 | 0x207 | Trigger | 2006 | 拨弹轮 |
| - | 0x209 | Yaw | GM6020 | Yaw 云台 |
| - | 0x20A | Pitch | GM6020 | Pitch 云台 |

> ⚠️ **已知冲突**: Fric2 和 Trigger 共享 ESC ID 7 (反馈 0x207)。Fric2 的 detect_hook 不会被调用，裁判系统可能读到 fric2 离线。但因两者都走 0x1FF 控制帧（非 0x200 独立 ID 帧），物理上不影响转动。

---

## 3. 软件架构

### 目录结构

```
├── Src/                    # STM32CubeMX 生成的驱动代码
│   ├── main.c              # 入口：HAL 初始化 → 外设 → FreeRTOS 启动
│   ├── freertos.c          # 任务创建（线程定义 + 优先级 + 栈大小）
│   ├── can.c               # CAN 初始化（⚠️ AutoBusOff=DISABLE）
│   └── stm32f4xx_it.c      # 中断服务函数
├── application/            # 核心应用层（所有任务在此）
│   ├── gimbal_task.c       # 云台控制任务（1kHz, Priority High）
│   ├── gimbal_behaviour.c  # 云台行为状态机（切换模式逻辑）
│   ├── chassis_task.c      # 底盘控制任务
│   ├── chassis_behaviour.c # 底盘行为状态机
│   ├── shoot.c             # 射击控制（摩擦轮 + 拨弹轮）
│   ├── CAN_receive.c       # CAN 接收 + 发送（核心）
│   ├── INS_task.c          # IMU 姿态解算（Priority Realtime）
│   ├── detect_task.c       # 离线检测（看门狗系统）
│   ├── remote_control.c    # 遥控器 SBUS 接收（DMA + 空闲中断）
│   ├── referee_usart_task.c # 裁判系统通信
│   ├── calibrate_task.c    # 云台校准任务
│   ├── led_flow_task.c     # LED 灯效
│   └── ...
├── components/             # 组件库
│   ├── algorithm/          # AHRS 姿态解算库 + PID 库
│   ├── controller/pid.c    # 标准 PID 实现
│   ├── devices/            # BMI088, IST8310, OLED 驱动
│   └── support/            # FIFO, CRC, 内存管理
├── bsp/boards/             # 板级外设驱动（激光、摩擦轮、蜂鸣器等）
├── Drivers/                # HAL 库 + CMSIS
└── Middlewares/             # FreeRTOS + USB
```

### 任务优先级 & 栈（从高到低）

| 任务 | 优先级 | 栈 (words) | 周期 | 功能 |
|------|--------|------------|------|------|
| INS_task | **Realtime** (最高) | 1024 | 1kHz | IMU 姿态解算 |
| gimbal_task | **High** | 512 | 1kHz | 云台 + 射击控制 |
| chassis_task | **AboveNormal** | 512 | 1kHz | 底盘控制 |
| detect_task | Normal | 256 | 10ms | 离线检测 |
| calibrate_task | Normal | 512 | - | 云台校准 |
| led/oled/usb/referee... | Normal~Low | 128~256 | - | 辅助功能 |

---

## 4. 核心概念详解

### 4.1 云台双模式控制

云台有**三种电机模式** + **多种行为模式**：

**电机模式** (`gimbal_motor_mode_e`):
| 模式 | 含义 | 反馈源 |
|------|------|--------|
| GIMBAL_MOTOR_RAW | 直接发电流值 | 无反馈 |
| GIMBAL_MOTOR_GYRO | 陀螺仪绝对角度控制 | IMU 欧拉角 |
| GIMBAL_MOTOR_ENCONDE | 编码器相对角度控制 | 电机码盘 |

**行为模式** (`gimbal_behaviour_e`):
- `GIMBAL_ZERO_FORCE` — 电机下电（s[1] 下档）
- `GIMBAL_INIT` — 初始化回中（过渡状态）
- `GIMBAL_ABSOLUTE_ANGLE` — 绝对角模式，陀螺仪控制（s[1] 中档）
- `GIMBAL_RELATIVE_ANGLE` — 相对角模式，编码器控制（s[1] 上档）
- `GIMBAL_CALI` — 校准模式
- `GIMBAL_MOTIONLESS` — 静止模式

**控制串级**:
```
角度环 (gimbal_PID_calc) → 输出目标角速度
  ↓
速度环 (PID_calc) → 输出电机电流
  ↓
CAN 发送 → GM6020 电机
```

### 4.2 两种角度测量方式

| | absolute_angle | relative_angle |
|------|------|------|
| 来源 | IMU (BMI088 姿态解算) | 电机编码器 (ecd - offset_ecd) |
| 参考系 | 重力水平面 | 上电位置 |
| 延迟 | 有滤波延迟 (~ms) | 几乎为零 |
| 噪声 | 较高 | 低 |
| 范围 | [-π, π] | [min_relative, max_relative] |

### 4.3 gimbal_PID_calc vs 标准 PID_calc

**gimbal_PID_calc** (云台外环角度 PID):
```c
Dout = KD * motor_gyro;  // D 项是陀螺仪角速度，不是误差微分！
out = KP*angle_err + KI*∫err + KD*gyro;
```
- **KD=1.0 的含义**: 陀螺仪速度完全前馈进速度环设定值，速度环的 gyro 反馈被抵消 → 失去阻尼
- **KD=0**: 速度环有 `-KP_speed * gyro` 的天然阻尼 → 可能因 gyro 噪声抖动

**标准 PID_calc** (速度环):
```c
Dout = Kd * (error[0] - error[1]);  // 误差微分（无滤波）
```

### 4.4 遥控器控制协议

- 基于 SBUS 协议，18 字节帧
- 5 个通道 (ch[0]~ch[4])，范围 364~1684
- 2 个开关 (s[0], s[1])，UP=1, DOWN=2, MID=3
- 鼠标数据 (x, y, z, 左键, 右键)
- 键盘数据 (bitmask, WASD + shift/ctrl + QERF etc.)
- DMA + 空闲中断接收，热插拔支持

### 4.5 离线检测系统

每个设备有时间戳记录，`detect_hook(ID)` 更新时间戳，检测任务定期检查：
- 超时 `offline_time` → 标记为离线上报
- 恢复后等待 `online_time` → 标记为在线

错误类型列表见 `detect_task.h:62-81`。

---

## 5. PID 参数速查表

### Pitch (GM6020)

| 模式 | KP | KI | KD | MAX_OUT | 速度环 KP |
|------|------|------|------|------|------|
| 绝对角 (GYRO) | 5.0 | 0.0 | 0.0 | 12.0 | 850 |
| 相对角 (ENCODE) | 18.0 | 0.0 | 1.0 | 12.0 | 850 |

### Yaw (GM6020)

| 模式 | KP | KI | KD | MAX_OUT | 速度环 KP |
|------|------|------|------|------|------|
| 绝对角 (GYRO) | 20.0 | 0.0 | 0.0 | 12.0 | 1000 |
| 相对角 (ENCODE) | 20.0 | 0.0 | 0.0 | 12.0 | 1000 |

### 摩擦轮 (C620, 开环斜坡)

| 参数 | 值 |
|------|------|
| FRIC_DOWN | 6000 |
| FRIC_UP | 9000 |
| SHOOT_FRIC_CURRENT_ADD_VALUE | 100 (斜坡步长) |

### 拨弹轮 (2006, 位置 PID)

| 参数 | 值 |
|------|------|
| KP | 800.0 |
| KI | 0.5 |
| KD | 0.0 |
| MAX_OUT | 10000 |

---

## 6. 已知问题

| # | 问题 | 状态 | 说明 |
|------|------|------|------|
| 1 | Fric2/Trigger 共享反馈 ID 0x207 | ⚠️ 物理无法修复 | Fric2 的 detect_hook 不会触发 |
| 2 | CAN AutoBusOff = DISABLE | ⚠️ 尝试修复失败 | 改为 ENABLE 后动不了，已还原 |
| 3 | Pitch 绝对角限位混合坐标系 | ❌ 误判 | bias_angle 的 IMU/encoder 偏移在减法中抵消，公式数学正确：relative+bias=rel_set |
| 4 | Pitch 绝对角 KP=5 偏软 | ⚠️ 待调 | 相对角 KP=18 工作正常 |
| 5 | Yaw 绝对角已绕过机械限位 | ✅ 已修复 | 滑环支持 360° |

---

## 7. 学员背景 & 学习目标

- 学员: 有 C 语言基础（非嵌入式），了解 CAN 通信原理，了解 RTOS 概念
- 目标: 熟练掌握步兵代码，能独立调试和修改
- 学习重点: 嵌入式 C 基础 → CAN 驱动 → PID 控制 → RTOS 任务 → 云台/底盘/射击系统

---

## 8. 学习路线

1. **嵌入式基础**: 寄存器、中断、DMA、HAL 库概念
2. **CAN 通信**: DJI 电机协议、0x200/0x1FF/0x2FF 帧格式、ID 分配
3. **PID 控制**: 位置式/增量式 PID、串级 PID、gimbal_PID_calc 的特殊 D 项
4. **FreeRTOS**: 任务创建、调度、优先级、中断与任务的交互
5. **子系统深入**: 云台 → 射击 → 底盘 → 裁判系统 → 校准
