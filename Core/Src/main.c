/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_comm.h"
#include "CAN_receive.h"
#include "Motor.h"
#include "remote_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ===== DT7 遥控器控制 Yaw 转速的参数 =====
 * 左拨杆 s[1]：档1 = 固定转速1，档2 = 固定转速2，档3 = 右摇杆控制。
 * 转速单位 rpm（GM6020 直驱，输出轴 = 转子转速），负值 = 反转。
 * 占位值，实车确认转向和手感后再调。 */
#define YAW_SPEED_1     150.0f
#define YAW_SPEED_2     60.0f

/* 右摇杆满推（ch 值 ±660）对应的转速，最终受 YAW_SPIN_MAX_RPM 限幅 */
#define YAW_STICK_SEN   300.0f

/* 摇杆中位死区：摇杆回中不一定精确为 0，不设死区会让 Yaw 一直慢慢转 */
#define RC_DEADBAND     60

/* ===== 主机模式：向水平轴板（板B）发送遥控数据的周期 ===== */
#define BT_SEND_PERIOD_MS  40U      /* 25Hz，够用且不占带宽 */

/* ===== yaw 控制拨杆（s[1]，物理左侧）的档位原始值 =====
 * 实测本遥控器是标准编码：上=1、中=3、下=2（中档是 3 不是 2）。
 * 功能分配：上=固定转速1，中=固定转速2，下=右摇杆控制。
 * 用这里的常量而不是标准车的 switch_is_xxx 宏，是为了让编码一目了然。 */
#define RC_SW_POS_UP    1
#define RC_SW_POS_MID   3
#define RC_SW_POS_DOWN  2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static motor_ctrl_t yaw_motor;
static uint8_t control_started;

/* TIM6 每 1ms 置 1，主循环看到它才执行一轮控制（volatile：中断里写） */
static volatile uint8_t control_tick_flag;

/* ===== 原「从机」模式：用蓝牙接收 PC 发来的转速指令 =====
 * 现在改成「主机」模式——USART1 改为向水平轴板（板B）发送遥控数据，
 * 这段接收代码用 #if 0 关掉保留。要恢复时注意：它和主机发送共用 USART1。 */
#if 0
#define BT_RX_BUF_SIZE 16
static volatile uint8_t bt_rx_buf[BT_RX_BUF_SIZE]; /* 行缓冲：中断里写，主循环读 */
static volatile uint8_t bt_rx_len;                 /* 当前行已收字节数 */
static volatile uint8_t bt_line_ready;             /* 1 = 收到完整一行，待主循环解析 */
static fp32 bt_target_rpm = 0.0f;                  /* 上电静止，等蓝牙发转速指令才动 */
static uint8_t rx_byte;
#endif

/* 本控制周期实际使用的目标转速（见 yaw_target_rpm） */
static fp32 yaw_rpm_cmd = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <string.h>

/**
 * @brief SerialPlot 输出：每 10ms 发一行 ASCII 数据。
 *        格式: target_rpm,actual_rpm,pid_output\n
 *        SerialPlot 设为 ASCII 模式、逗号分隔、3 通道。
 *        USART1 TX (PA9) → USB-TTL RX，115200 8N1。
 *        阻塞发送 ~30 字节约 2.6ms，每 10ms 一次不影响 1ms 控制周期。
 */
void serialplot_output(fp32 target, fp32 actual, int16_t pid_out)
{
  static uint32_t last_tick = 0;
  uint32_t now = HAL_GetTick();
  char buf[48];
  int len;

  if (now - last_tick < 10U)
    return;
  last_tick = now;

  len = snprintf(buf, sizeof(buf), "%.1f,%.1f,%d\n",
                 (double)target, (double)actual, (int)pid_out);
  if (len > 0)
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 5U);
}

/* ===== 原「从机」模式：解析 PC 发来的转速指令（已随从机模式一起关掉）======
 *
 * 解析蓝牙一行转速指令：[可选负号]+数字，如 "60"、"-120"、"0"。
 * 解析成功则更新 bt_target_rpm，并限幅到 ±YAW_SPIN_MAX_RPM。
 */
#if 0
static void bt_parse_speed_line(void)
{
  int32_t value = 0;
  int32_t sign = 1;
  uint8_t i = 0U;
  uint8_t valid = 1U;

  if (bt_rx_buf[0] == '-')
  {
    sign = -1;
    i = 1U;
  }
  if (i >= bt_rx_len)               /* 只有负号没有数字 */
    valid = 0U;

  for (; i < bt_rx_len; i++)
  {
    if (bt_rx_buf[i] < '0' || bt_rx_buf[i] > '9')
    {
      valid = 0U;                   /* 非数字字符：整行丢弃 */
      break;
    }
    value = value * 10 + (bt_rx_buf[i] - '0');
    if (value > 1000000)            /* 超长数字防溢出：提前截断，后面统一限幅 */
      break;
  }

  if (valid)
  {
    value = sign * value;
    if (value > YAW_SPIN_MAX_RPM)   /* 限幅：超出安全范围压回边界 */
      value = YAW_SPIN_MAX_RPM;
    if (value < -YAW_SPIN_MAX_RPM)
      value = -YAW_SPIN_MAX_RPM;
    bt_target_rpm = (fp32)value;
  }

  /* 先清长度再清标志：标志没清前中断不会写入新数据 */
  bt_rx_len = 0U;
  bt_line_ready = 0U;
}
#endif

/**
 * @brief 主机模式：向水平轴板（板B）发送一帧遥控数据，25Hz。
 *
 * 帧格式（ASCII）：
 *   $B,<sw>,<stick>,<en>\r\n
 *     sw    水平轴拨杆档位（右拨杆 s[0]）：1/2/3，0 = 无效
 *     stick 水平轴摇杆值，归一化到 -1000~1000（左摇杆横轴 ch[2]）
 *     en    遥控器在线标志：1 = 在线，0 = 离线（板B 收到 0 应停机）
 *
 * 选 ASCII 而不是二进制，是为了板B 能脱离遥控器、用 PC 串口助手直接灌帧调试。
 * 板A 只做搬运，模式解释和限位全部留在板B 本地。
 *
 * ⚠️ USART1 TX 上同时还有 serialplot_output() 的遥测数据，两者共用一条线。
 *    板B 的解析必须靠 "$B" 帧头过滤；定型时可以把 serialplot 关掉。
 */
static void bt_send_rc_frame(void)
{
  static uint32_t last_tick = 0U;
  const RC_ctrl_t *rc;
  char buf[32];
  int len;
  int32_t stick;

  if (HAL_GetTick() - last_tick < BT_SEND_PERIOD_MS)
    return;
  last_tick = HAL_GetTick();

  rc = get_remote_control_point();

  /* 左摇杆横轴：先过中位死区，再归一化到 ±1000。
   * 死区是必须的——摇杆回中不一定精确为 0，不过死区的话板B 手动档会一直慢慢漂。 */
  if (rc->rc.ch[2] > -RC_DEADBAND && rc->rc.ch[2] < RC_DEADBAND)
    stick = 0;
  else
    stick = (int32_t)rc->rc.ch[2] * 1000 / RC_CH_VALUE_MAX_ABS;

  len = snprintf(buf, sizeof(buf), "$B,%d,%d,%d\r\n",
                 (int)rc->rc.s[0],                    /* 档位：右拨杆 */
                 (int)stick,
                 RC_is_offline() ? 0 : 1);
  if (len > 0)
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 5U);
}

/**
 * @brief 依据 DT7 左拨杆和右摇杆算出 Yaw 目标转速。
 *
 * 左拨杆 s[1]：档1 = YAW_SPEED_1，档2 = YAW_SPEED_2，档3 = 右摇杆 ch[1]。
 *
 * 上电档位不生效：上电后先记住拨杆停在哪个档，必须等档位变化一次才解锁，
 * 避免一上电 Yaw 就按固定转速转起来。遥控器离线会清掉解锁状态，
 * 重新上线后要再拨一次拨杆。
 *
 * 遥控器离线时停转（原「回落到 PC 蓝牙指令」的兜底随从机模式一起关掉了）。
 *
 * @retval 目标转速 (rpm)，已限幅到 ±YAW_SPIN_MAX_RPM
 */
static fp32 yaw_target_rpm(void)
{
  const RC_ctrl_t *rc;
  static uint8_t boot_s1;         /* 上电后第一帧记录的档位 */
  static uint8_t boot_s1_valid;
  static uint8_t armed;           /* 档位变化过一次才置 1 */
  int16_t stick;
  fp32 rpm;

  if (RC_is_offline())
  {
    boot_s1_valid = 0U;
    armed = 0U;
    return 0.0f;
  }

  rc = get_remote_control_point();

  /* 上电第一帧：只记档位，不动作 */
  if (!boot_s1_valid)
  {
    boot_s1 = (uint8_t)rc->rc.s[1];
    boot_s1_valid = 1U;
    return 0.0f;
  }

  /* 档位变过才解锁；解锁前保持在 0 */
  if (!armed)
  {
    if ((uint8_t)rc->rc.s[1] != boot_s1)
      armed = 1U;
    return 0.0f;
  }

  /* 档位 → 功能：上=固定转速1，中=固定转速2，下=右摇杆控制。
   * 这里用实际原始值比较，不用标准车的 switch_is_xxx 宏（那套编码和实物不符）。 */
  if ((uint8_t)rc->rc.s[1] == RC_SW_POS_UP)         /* 上档 */
  {
    rpm = YAW_SPEED_1;
  }
  else if ((uint8_t)rc->rc.s[1] == RC_SW_POS_MID)   /* 中档 */
  {
    rpm = YAW_SPEED_2;
  }
  else                                              /* 下档：右摇杆 */
  {
    /* 右摇杆垂直通道，中位死区内归零 */
    stick = rc->rc.ch[1];
    if (stick > -RC_DEADBAND && stick < RC_DEADBAND)
      rpm = 0.0f;
    else
      rpm = (fp32)stick * YAW_STICK_SEN / (fp32)RC_CH_VALUE_MAX_ABS;
  }

  if (rpm > (fp32)YAW_SPIN_MAX_RPM)
    rpm = (fp32)YAW_SPIN_MAX_RPM;
  if (rpm < -(fp32)YAW_SPIN_MAX_RPM)
    rpm = -(fp32)YAW_SPIN_MAX_RPM;

  return rpm;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  if (can_comm_init() != HAL_OK)
    Error_Handler();

  motor_ctrl_init(&yaw_motor, MOTOR_AXIS_YAW);
  control_started = 0U;
  control_tick_flag = 0U;

  /* 启动 TIM6 中断：每 1ms 置 control_tick_flag，驱动控制周期 */
  HAL_TIM_Base_Start_IT(&htim6);

  /* 原「从机」模式：等蓝牙发转速指令（已关闭 —— USART1 现在用作主机发送）
   * HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
   */

  /* 启动 DT7/DR16 遥控器接收：USART3 + DMA 双缓冲 + 空闲中断 */
  remote_control_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
    /* 等 TIM6 的 1ms 节拍：没到就空转（不阻塞，让出 CPU 给中断） */
    if (!control_tick_flag)
      continue;
    control_tick_flag = 0U;

    /* 主机模式：把遥控器数据发给水平轴板（板B）。25Hz 限流在函数内部 */
    bt_send_rc_frame();

    /* 本周期目标转速：来自 DT7；遥控器离线则为 0 */
    yaw_rpm_cmd = yaw_target_rpm();

    /* 上电延迟 500ms 等 CAN 反馈稳定 */
    if (HAL_GetTick() < 500U)
    {
      CAN_cmd_yaw(0);
      continue;
    }

    if (!CAN_motor_feedback_ready())  /* 反馈丢失：立即发 0 保安全，清理状态，下次恢复后重新配置 */
    {
      CAN_cmd_yaw(0);
      if (control_started)
      {
        motor_ctrl_clear(&yaw_motor);
        control_started = 0U;
      }
      continue;
    }

    /* 目标转速 0：发 0 电压后直接等下一周期。
     *
     * 只清速度环积分、不清编码器状态：motor_ctrl_update 在 spin_speed_rpm=0 时
     * 不调用 PID 就返回 0，积分会冻结在停转前的值，不清的话下次启动输出会突跳。
     * 而 motor_ctrl_clear 会连坐标系一起重置——DT7 模式下摇杆回中很频繁，
     * 每回中一次就重建坐标系没有意义。 */
    if (yaw_rpm_cmd == 0.0f)
    {
      CAN_cmd_yaw(0);
      Pid_clear(&yaw_motor.pid_speed);
      continue;
    }

    if (!control_started)   /* 先捕获首帧反馈，记 0° 参考 */
    {
      motor_ctrl_update(&yaw_motor, 0.0f,
                        motor_measure[YAW_MOTOR].ecd,
                        motor_measure[YAW_MOTOR].speed_rpm);
      control_started = 1U;
      CAN_cmd_yaw(0);
      continue;
    }

    {
      /* 每周期刷新匀速设定：运行中收到新转速立即生效 */
      motor_ctrl_set_spin(&yaw_motor, yaw_rpm_cmd, 0U);

      /* yaw 匀速模式：target_angle 被忽略，传 0.0f 即可 */
      int16_t yaw_v = motor_ctrl_update(&yaw_motor, 0.0f,    /* 主要循环控制 */
                                        motor_measure[YAW_MOTOR].ecd,
                                        motor_measure[YAW_MOTOR].speed_rpm);
      CAN_cmd_yaw(yaw_v);

      /* SerialPlot 输出：目标转速, 实际转速, PID 输出 */
      serialplot_output(yaw_rpm_cmd,
                        (fp32)motor_measure[YAW_MOTOR].speed_rpm,
                        yaw_v);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* TIM6 更新中断回调：每 1ms 置控制节拍标志（中断里只置标志，不做控制） */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
    control_tick_flag = 1U;
}
#if 0
/* ===== 原「从机」模式：蓝牙逐字节接收回调（已关闭）=====
 * 逐字节收进行缓冲，收到回车/换行置一行完成标志。
 * 指令格式：一行一个转速数字（可带负号），如 "60"、"-120"、"0"，
 * 由主循环解析并限幅到 ±YAW_SPIN_MAX_RPM。
 * 注意：USART1 现在用作主机发送，恢复这段前要先停掉 bt_send_rc_frame()。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    HAL_UART_Transmit(&huart1, &rx_byte, 1, 10);  /* 调试回显：收到什么原样发回，验证蓝牙链路 */
    if (rx_byte == '\r' || rx_byte == '\n')
    {
      if (bt_rx_len > 0U && !bt_line_ready)
        bt_line_ready = 1U;                       /* 有内容才算一行 */
    }
    else if (!bt_line_ready && bt_rx_len < BT_RX_BUF_SIZE - 1U)
    {
      bt_rx_buf[bt_rx_len] = rx_byte;            /* 累积数字/负号 */
      bt_rx_len++;
    }
    /* 上一行还没处理或缓冲满：多余字节丢弃（简单保护） */
    /* 重新启动接收，等待下一字节 */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}
#endif
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
