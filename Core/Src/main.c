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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_comm.h"
#include "CAN_receive.h"
#include "Motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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

/* Bluetooth speed command (one number per line, ends with \r or \n):
 * "60"=60rpm forward, "-120"=120rpm reverse, "0"=stop. Clamped to ±YAW_SPIN_MAX_RPM. */
#define BT_RX_BUF_SIZE 16
static volatile uint8_t bt_rx_buf[BT_RX_BUF_SIZE]; /* 行缓冲：中断里写，主循环读 */
static volatile uint8_t bt_rx_len;                 /* 当前行已收字节数 */
static volatile uint8_t bt_line_ready;             /* 1 = 收到完整一行，待主循环解析 */
static fp32 bt_target_rpm = 0.0f;                  /* 上电静止，等蓝牙发转速指令才动 */
static uint8_t rx_byte;

/* CAN 诊断观测变量（Keil Debug → Watch 直接看，10 秒后暂停读数）：
 * g_dbg_given_current 是核心：反馈帧里的"实际转矩电流"。
 *   一直≈0        → 电机根本没执行我们的指令（电机侧配置问题）
 *   |值|>1500     → 电机在真实拉电流（执行了，看 g_dbg_ecd）
 * g_dbg_ecd 持续变化 → 转子在转（哪怕机构看着没动 = 传动脱开）
 * g_dbg_rx_cnt 大约每秒 +1000 → 反馈正常在收 */
volatile uint32_t g_dbg_phase = 0;          /* 当前测试相位 ms（0~6000 循环） */
volatile uint32_t g_dbg_tx_ok = 0;          /* 成功入队的控制帧数 */
volatile uint32_t g_dbg_tx_fail = 0;        /* 入队失败次数（邮箱满） */
volatile uint32_t g_dbg_rx_cnt = 0;         /* 收到的反馈帧计数 */
volatile int16_t  g_dbg_speed_rpm = 0;      /* 电机反馈转速 */
volatile int16_t  g_dbg_given_current = 0;  /* 电机反馈实际转矩电流（关键） */
volatile uint16_t g_dbg_ecd = 0;            /* 编码器原始值（持续变 = 转子在转） */
volatile uint8_t  g_dbg_temp = 0;           /* 电机温度 */
volatile uint16_t g_dbg_max_current = 0;    /* 历史最大 |转矩电流|（暂停时机不限，漏不掉） */
volatile uint32_t g_dbg_ecd_moves = 0;      /* 编码器发生变化的次数（>0 = 转子动过） */
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

/**
 * @brief 解析蓝牙一行转速指令：[可选负号]+数字，如 "60"、"-120"、"0"。
 *        解析成功则更新 bt_target_rpm，并限幅到 ±YAW_SPIN_MAX_RPM；
 *        含非法字符的行整行丢弃，维持原转速不变。
 */
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
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  if (can_comm_init() != HAL_OK)
    Error_Handler();

  motor_ctrl_init(&yaw_motor, MOTOR_AXIS_YAW);
  control_started = 0U;
  control_tick_flag = 0U;

  /* 启动 TIM6 中断：每 1ms 置 control_tick_flag，驱动控制周期 */
  HAL_TIM_Base_Start_IT(&htim6);

  /* Start USART1 RX interrupt (wait for Bluetooth command) */
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 自动分相测试（6 秒一个周期，反复循环）：
     *   0~2s   电压帧 0x1FF = 15000（60% 占空，电压模式下必转）
     *   2~3s   停（发 0）
     *   3~5s   电流帧 0x1FE = 4000（约 0.73A，仅电流环开启的电机响应）
     *   5~6s   停（发 0）
     * 电机什么时候动 = 答案：
     *   第 0~2s 动   → 电压模式正常，之前是电压太小/闭环参数问题
     *   第 3~5s 动   → 电机其实开着电流环（绿灯闪只是没报警，固件版本行为）
     *   全程不动     → 发送方向物理层问题（对照红灯）
     * C 板 LED（单颗 RGB，颜色会混）：
     *   绿/青（绿蓝混）= 帧发得出去（有 ACK）
     *   紫/洋红（红蓝混）= 发送邮箱卡死，帧无人应答
     *   纯蓝 = 只收到反馈，两种发送状态都没亮（异常）
     * 调完后改回 #if 0 恢复正常逻辑。 */
#if 0
    if (!control_tick_flag)
      continue;
    control_tick_flag = 0U;

    {
      uint32_t phase = HAL_GetTick() % 6000U;
      uint32_t free_mb = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
      HAL_StatusTypeDef tx_ret = HAL_OK;
      static uint32_t last_tick_seen = 0U;

      g_dbg_phase = phase;

      if (phase < 2000U)
        tx_ret = CAN_cmd_yaw(15000);         /* A：电压帧 0x1FF = 15000 */
      else if (phase < 3000U)
        tx_ret = CAN_cmd_yaw(0);             /* 停 */
      else if (phase < 5000U)
        tx_ret = CAN_cmd_yaw_current(4000);  /* B：电流帧 0x1FE = 4000 */
      else
        tx_ret = CAN_cmd_yaw(0);             /* 停 */

      if (tx_ret == HAL_OK)
        g_dbg_tx_ok++;
      else
        g_dbg_tx_fail++;

      /* 反馈帧计数：每收一帧 last_rx_tick 都会变 */
      if (motor_measure[YAW_MOTOR].last_rx_tick != last_tick_seen)
      {
        g_dbg_rx_cnt++;
        last_tick_seen = motor_measure[YAW_MOTOR].last_rx_tick;
      }

      g_dbg_speed_rpm = motor_measure[YAW_MOTOR].speed_rpm;
      g_dbg_given_current = motor_measure[YAW_MOTOR].given_current;
      g_dbg_ecd = motor_measure[YAW_MOTOR].ecd;
      g_dbg_temp = motor_measure[YAW_MOTOR].temperature;

      {
        /* 统计量：暂停读数不挑时机 */
        uint16_t abs_cur;
        static uint16_t last_ecd = 0U;

        abs_cur = (g_dbg_given_current < 0)
                    ? (uint16_t)(-g_dbg_given_current)
                    : (uint16_t)g_dbg_given_current;
        if (abs_cur > g_dbg_max_current)
          g_dbg_max_current = abs_cur;

        if (g_dbg_ecd != last_ecd)
        {
          g_dbg_ecd_moves++;
          last_ecd = g_dbg_ecd;
        }
      }

      /* LED 诊断（C 板单颗 RGB）：
       * 蓝灯 = 反馈在收
       * 绿灯 = 发送正常（上一帧已离开邮箱，总线有 ACK）
       * 红灯 = 电机在真实拉电流 |转矩电流|>1500（即指令被执行） */
      if (motor_measure[YAW_MOTOR].received)
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET); /* Blue ON */
      else
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);   /* Blue OFF */

      if (free_mb != 0U)
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET); /* Green ON */
      else
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);   /* Green OFF */

      if (g_dbg_given_current > 1500 || g_dbg_given_current < -1500)
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET); /* Red ON */
      else
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);   /* Red OFF */
    }
    continue;
#endif

    /* 等 TIM6 的 1ms 节拍：没到就空转（不阻塞，让出 CPU 给中断） */
    if (!control_tick_flag)
      continue;
    control_tick_flag = 0U;

    /* 解析蓝牙一行转速指令：数字 = rpm（负号 = 反转），限幅 ±YAW_SPIN_MAX_RPM */
    if (bt_line_ready)
      bt_parse_speed_line();


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

    if (bt_target_rpm == 0.0f)   /* 目标转速 0：发 0 电压，等待指令 */
    {
      CAN_cmd_yaw(0);
      if (control_started)
      {
        motor_ctrl_clear(&yaw_motor);
        control_started = 0U;
      }
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
      motor_ctrl_set_spin(&yaw_motor, bt_target_rpm, 0U);

      /* yaw 匀速模式：target_angle 被忽略，传 0.0f 即可 */
      int16_t yaw_v = motor_ctrl_update(&yaw_motor, 0.0f,    /* 主要循环控制 */
                                        motor_measure[YAW_MOTOR].ecd,
                                        motor_measure[YAW_MOTOR].speed_rpm);
      CAN_cmd_yaw(yaw_v);

      /* SerialPlot 输出：目标转速, 实际转速, PID 输出 */
      serialplot_output(bt_target_rpm,
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
/* 串口接收完成回调：逐字节收进行缓冲，收到回车/换行置一行完成标志。
 * 指令格式：一行一个转速数字（可带负号），如 "60"、"-120"、"0"，
 * 由主循环解析并限幅到 ±YAW_SPIN_MAX_RPM。 */
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
      bt_rx_buf[bt_rx_len] = rx_byte;             /* 累积数字/负号 */
      bt_rx_len++;
    }
    /* 上一行还没处理或缓冲满：多余字节丢弃（简单保护） */
    /* 重新启动接收，等待下一字节 */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}
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
  * @param  file: pointer to the source file name.
  * @param  line: assert_param error line source number.
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
