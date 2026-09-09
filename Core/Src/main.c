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
#include "trajectory.h"
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
static motor_ctrl_t horizontal_motor;
static uint8_t control_started;

/* TIM6 每 1ms 置 1，主循环看到它才执行一轮控制（volatile：中断里写） */
static volatile uint8_t control_tick_flag;

/* Bluetooth start/stop control: 1=run, 0=stop, default stop on power-up */
static volatile uint8_t bt_run_command = 0U;  /* 蓝牙控制：1=启动 0=停止，上电默认停止 */
static uint8_t rx_byte;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  motor_ctrl_init(&horizontal_motor, MOTOR_AXIS_HORIZONTAL);
  trajectory_init();
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
    /* 等 TIM6 的 1ms 节拍：没到就空转（不阻塞，让出 CPU 给中断） */
    if (!control_tick_flag)
      continue;
    control_tick_flag = 0U;

    if (!CAN_motor_feedback_ready())  // 反馈丢失：立即发 0 保安全，清理状态，下次恢复后重新配置
    {
      CAN_cmd_both(0, 0);
      if (control_started)
      {
        motor_ctrl_clear(&yaw_motor);
        motor_ctrl_clear(&horizontal_motor);
        trajectory_init();
        control_started = 0U;
      }
      continue;
    }

    if (!bt_run_command)   /* 蓝牙未发启动指令：发 0 电压，等待 */
    {
      CAN_cmd_both(0, 0);
      if (control_started)
      {
        motor_ctrl_clear(&yaw_motor);
        motor_ctrl_clear(&horizontal_motor);
        trajectory_init();
        control_started = 0U;
      }
      continue;
    }

    if (!control_started)   //先配置
    {
      motor_ctrl_update(&yaw_motor, 0.0f,
                        motor_measure[YAW_MOTOR].ecd,
                        motor_measure[YAW_MOTOR].speed_rpm);
      motor_ctrl_update(&horizontal_motor, 0.0f,
                        motor_measure[HORIZONTAL_MOTOR].ecd,
                        motor_measure[HORIZONTAL_MOTOR].speed_rpm);
      /* yaw 匀速不走轨迹，set_yaw_mode 无实际效果（保留备将来切换角度模式） */
      // trajectory_set_yaw_mode(TRAJ_STOP);
      motor_ctrl_set_spin(&yaw_motor, YAW_SPIN_SPEED_RPM, 0U); /* yaw 默认匀速连续转（不限位） */
      trajectory_set_horizontal_mode(TRAJ_TRIANGLE); /* 水平轴单环+三角波：限位内匀速往返扫摆 */
      control_started = 1U;
      CAN_cmd_both(0, 0);
      continue;
    }

    {
      /* yaw 匀速模式：target_angle 被忽略，传 0.0f 即可 */
      int16_t yaw_v = motor_ctrl_update(&yaw_motor, 0.0f,     //主要循环控制
                                        motor_measure[YAW_MOTOR].ecd,
                                        motor_measure[YAW_MOTOR].speed_rpm);
      int16_t hori_v = motor_ctrl_update(&horizontal_motor,
                                         trajectory_get_horizontal(),
                                         motor_measure[HORIZONTAL_MOTOR].ecd,
                                         motor_measure[HORIZONTAL_MOTOR].speed_rpm);
      CAN_cmd_both(yaw_v, hori_v);
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
/* 串口接收完成回调：收到一个字节，处理蓝牙指令 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    HAL_UART_Transmit(&huart1, &rx_byte, 1, 10);  /* 调试回显：收到什么原样发回，验证蓝牙链路 */
    if (rx_byte == '1')
      bt_run_command = 1U;           /* 启动 */
    else if (rx_byte == '0')
      bt_run_command = 0U;           /* 停止 */
    /* 重新启动接收，等待下一条指令 */
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
