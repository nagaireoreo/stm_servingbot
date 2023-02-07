/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "Encoder.h"
#include "PID.h"
#include "Serial.h"
#include "CommonMath.h"
#include "stdio.h"
#include "string.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define ACCELELATED_ADD_VEL 0.03
/* USER CODE BEGIN PD */
// プロトタイプ宣言

// @brief	: 各車輪のデューティ比を設定する
int SetDuty(float p_duty_front, float p_duty_left, float p_duty_right);
// @brief	: 速度指令値をPWMのコンペア値に変換する（GM6020用）
int ConvertSpeedControlValue2PwmCompareCount(float p_speed_control_value);
// @brief	: 1byteを上位8bitと下位8bitに変換する
void ConvertByte2HighLowBit(int p_byte, int* p_low_bit, int* p_high_bit);
// @brief	: CAN1で送信する
int CAN1Transmit(uint32_t p_target_can_id, uint8_t p_tx_data[8]);
// @brief : define.hで設定した値から，全方向で出せる車体の最大速度を求める
float MaxRobotSpeed();
// @brief	: 正面ホイール側の回転速度を設定する
void SetPWMFront(float p_speed_command_value);
// @brief	: 左後ホイール側の回転速度を設定する
void SetPWMBackLeft(float p_speed_command_value);
// @brief	: 右後ホイール側の回転速度を設定する
void SetPWMBackRight(float p_speed_command_value);
// @brief : CAN通信のフィルタ設定をする
void CAN_FilterInit(void);


// @brief	: 加速度を考慮した速度を計算する
vector<float> CalcAcceleratedCmdValues(vector<float>, vector<float> , float);


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool g_is_up_control_cycle = false;
char g_received_char;
int k=0;
string uart_str;
//Serial serial;
int indexRead = 0;
// CAN
/*
uint32_t id;
uint32_t dlc;
uint8_t id517_readed_can_data[8];
uint8_t id518_readed_can_data[8];
uint8_t id519_readed_can_data[8];
int cnt = 0;
int id_1_cnt = 0;
int id_2_cnt = 0;
int id_3_cnt = 0;
*/
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM12_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_TIM14_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */


  string  msg;
  //msg = "Initialization End\r\n";
  //HAL_UART_Transmit( &huart2, (uint8_t *)msg.c_str(), msg.size(), 0xFFFF);

  // Encoder
  Encoder encoder;
  encoder.SetDriveEncoderPPR(14);
  encoder.SetDriveWheelDiameter(0.08);
  encoder.SetGearRatio(1.0/24.9);

  //msg = "Encoder Setting End\r\n";
  //HAL_UART_Transmit( &huart2, (uint8_t *)msg.c_str(), msg.size(), 0xFFFF);



  //msg = "PID Setting End\r\n";
  //HAL_UART_Transmit( &huart2, (uint8_t *)msg.c_str(), msg.size(), 0xFFFF);

  // PWM
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  HAL_Delay(10);
  // 安全のため初期デューティ比を0にする
  //__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0); // MD1
  //__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0); // MD2
  //__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0); // MD3

  // モータのPWMが指定の範囲外になったときにエラーになるから，立ち上げ時に0を送ってエラーリセットする
  SetPWMFront(0);
  SetPWMBackLeft(0);
  SetPWMBackRight(0);

  //__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, (int)(60000*0.05));
  //__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, (int)(60000*0.075));
  HAL_Delay(100);

  //msg = "PWM Setting End\r\n";
  //HAL_UART_Transmit( &huart2, (uint8_t *)msg.c_str(), msg.size(), 0xFFFF);

  // シリアル通信
  // DWA受信
  //HAL_UART_Receive_DMA(&huart2,serialData,DATANUM);
  Serial serial;
  // rplidarが遅延するの暫定対策
  int serial_count = 0;

  // CAN
  // CANのフィルタ設定
  CAN_FilterInit();
  HAL_CAN_Start(&hcan1);
  // 割り込み有効
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  msg = "CAN Setting Error ....... \r\n";
	  HAL_UART_Transmit( &huart2, (uint8_t *)msg.c_str(), msg.size(), 0xFFFF);
      Error_Handler();
  }
  //msg = "CAN Setting End\r\n";
  //HAL_UART_Transmit( &huart2, (uint8_t *)msg.c_str(), msg.size(), 0xFFFF);


  // Timer Interrupt
  HAL_TIM_Base_Start_IT(&htim6);

  HAL_UART_Transmit( &huart2, (uint8_t *)"start!\n", 7, 0xFFFF);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // エンコーダの初期値を初期化する
  InitEncoder();

  vector<float> accelerated_command_values;
  accelerated_command_values.push_back(0);
  accelerated_command_values.push_back(0);
  accelerated_command_values.push_back(0);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 // 10ms circle process
	if(g_is_up_control_cycle == true)
	{
      g_is_up_control_cycle = false;

      // < Encoder >
       encoder.Update();

      // 自己位置を取得する
      struct_posture_t pos = encoder.GetPositions();

      // シリアルの速度指令値を取得する
      serial.Update();

      string serial_str;

      // 速度指令値を取得する
      vector<float> target_speed_control_local_values = serial.GetSpeedControlValues(); // ロボット座標系

      // 加減速を考慮させる
      accelerated_command_values = CalcAcceleratedCmdValues(accelerated_command_values, target_speed_control_local_values, ACCELELATED_ADD_VEL);
      vector<float> speed_control_local_values = accelerated_command_values;


      //char msgc[50];
      //sprintf(msgc, "%f %f %f \r\n", target_speed_control_local_values[0], accelerated_command_values[0], speed_control_local_values[0]);
      //HAL_UART_Transmit( &huart2, (uint8_t *)msgc, strlen(msgc) + 1, 0xFFFF);
      //HAL_UART_Transmit( &huart2, (uint8_t *)"hello", 10, 0xFFFF);

      // < 速度指令値をロボット座標系からマップ座標系に変換する >
      vector<float> speed_control_global_values(3, 0); // マップ座標系

      // 変換する場合
      //Rotation(speed_control_local_values[0], speed_control_local_values[1], pos.angle, &speed_control_global_values[0], &speed_control_global_values[1]);
      //speed_control_global_values[2] = speed_control_local_values[2];

      // 変換しない場合
      speed_control_global_values[0] = speed_control_local_values[0];
      speed_control_global_values[1] = speed_control_local_values[1];
      speed_control_global_values[2] = speed_control_local_values[2];

      // 速度指令値から各車輪の目標速度を計算する
      vector<float> wheel_target_velocities(MOTOR_QTY, 0);
      ConvertBodyVel2WheelVelManuaRate(speed_control_global_values[0], speed_control_global_values[1], speed_control_global_values[2], &wheel_target_velocities[0], &wheel_target_velocities[1], &wheel_target_velocities[2], 0.8, 0.2);
      //ConvertBodyVel2WheelVel(speed_control_global_values[0], speed_control_global_values[1], speed_control_global_values[2], &wheel_target_velocities[0], &wheel_target_velocities[1], &wheel_target_velocities[2] );

      // < MD内の閉ループ制御を使う場合(PWMで指令値送信) >
      SetPWMFront(wheel_target_velocities[0]);
      SetPWMBackLeft(wheel_target_velocities[1]);
      SetPWMBackRight(wheel_target_velocities[2]);


      // GM6020キャリブレーション
//      if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==GPIO_PIN_RESET){
//			//HAL_UART_Transmit( &huart2, (uint8_t *)"ON\n", strlen("ON\n") + 1, 0xFFFF);
//			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 3000); // 起動時
//			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 3000); // 起動時
//			__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 3000); // 起動時
//		}else{
//			//HAL_UART_Transmit( &huart2, (uint8_t *)"OFF\n", strlen("OFF\n") + 1, 0xFFFF);
//			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 6000);
//			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 6000);
//			__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 6000);
//		}

	}
  }
  /* USER CODE END 3 */
}

/**
 * .
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	g_is_up_control_cycle = true;
	//if (htim == &htim6)
	//{
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, a%2);
	//}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance != USART2)
	{
		return;
	}
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &g_received_char, 1);
}





void CAN_FilterInit(void)
{
	CAN_FilterTypeDef filter;
	filter.FilterIdHigh         = 0;                        // フィルターID(上位16ビット)
	filter.FilterIdLow          = 0;                        // フィルターID(下位16ビット)
	filter.FilterMaskIdHigh     = 0;                        // フィルターマスク(上位16ビット)
	filter.FilterMaskIdLow      = 0;                        // フィルターマスク(下位16ビット)
	filter.FilterScale          = CAN_FILTERSCALE_32BIT;    // フィルタースケール
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;         // フィルターに割り当てるFIFO
	filter.FilterBank           = 0;                        // フィルターバンクNo
	filter.FilterMode           = CAN_FILTERMODE_IDMASK;    // フィルターモード
	filter.SlaveStartFilterBank = 14;                       // スレーブCANの開始フィルターバンクNo
	filter.FilterActivation     = ENABLE;                   // フィルター無効／有効
	if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
	{
		Error_Handler();
	}
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 各車輪のデューティ比を設定する
// @date	: 2021/05/2
//
// @param1[in]		: 前車輪のデューティ比(-1.0 ~ 1.0)
// @param1[in]		: 左車輪のデューティ比(-1.0 ~ 1.0)
// @param1[in]		: 右車輪のデューティ比(-1.0 ~ 1.0)
// @return			: 成功判定
//---------------------------------------------------------------------------------------------------------------------
int SetDuty(float p_duty_front, float p_duty_left, float p_duty_right)
{
	// -1.0 ~ 1.0 の間でクリッピングする
	p_duty_front = Clipping(p_duty_front, -1.0, 1.0);
	p_duty_left   = Clipping(p_duty_left,    -1.0, 1.0);
	p_duty_right = Clipping(p_duty_right, -1.0, 1.0);

	// 送信するコマンドが-30,000 ~ 30,000の範囲なので変換する
	int volutage_command_value_front = int(p_duty_front * 30000.0);
	int volutage_command_value_left   = int(p_duty_left    * 30000.0);
	int volutage_command_value_right = int(p_duty_right * 30000.0);
	// 正の値で正転するように負の値をかける
	volutage_command_value_front *=  -1;
	volutage_command_value_left   *= -1;
	volutage_command_value_right *= -1;


	// 1byteを上位8bitと下位8bitに変換する
	int command_low_bit_front,	command_high_bit_front;
	int command_low_bit_left,		command_high_bit_left;
	int command_low_bit_right,	command_high_bit_right;

	ConvertByte2HighLowBit(volutage_command_value_front, &command_low_bit_front, &command_high_bit_front);
	ConvertByte2HighLowBit(volutage_command_value_left,   &command_low_bit_left,    &command_high_bit_left);
	ConvertByte2HighLowBit(volutage_command_value_right, &command_low_bit_right, &command_high_bit_right);

	// 送信データを整列する
	uint8_t tx_data[8];
	tx_data[0] = command_high_bit_front;
	tx_data[1] = command_low_bit_front;
	tx_data[2] = command_high_bit_left;
	tx_data[3] = command_low_bit_left;
	tx_data[4] = 0xff;
	tx_data[5] = 0xff;
	tx_data[6] = command_high_bit_right;
	tx_data[7] = command_low_bit_right;

	// CAN ID
	uint32_t target_can_id = 0x1ff;

	// 送信する
	CAN1Transmit(target_can_id, tx_data);

}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 1byteを上位8bitと下位8bitに変換する
// @date	: 2021/04/29
//
// @param1[in]		: 変換したい1byte分の数値
// @param2[out]	: 変換後の下位8bit
// @param3[out]	: 変換したい1byte分の数値
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void ConvertByte2HighLowBit(int p_byte, int* p_low_bit, int* p_high_bit)
{
	int low_bit  = p_byte & 0b0000000011111111;
	int high_bit = p_byte & 0b1111111100000000;
	high_bit = high_bit >> 8;

	*p_low_bit  = low_bit;
	*p_high_bit = high_bit;
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: CAN1で送信する(データ長は8byte)
// @date	: 2021/04/29
//
// @param1[in]	: 速度指令値(-1.0 ~ 1.0)
// @return			: 成功判定
//---------------------------------------------------------------------------------------------------------------------
int CAN1Transmit(uint32_t p_target_can_id, uint8_t p_tx_data[8])
{
	  CAN_TxHeaderTypeDef TxHeader;
	  uint32_t TxMailbox;
	  //uint8_t TxData[8];

	  int res = 0;

	  if(0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
	  {
		  //HAL_UART_Transmit( &huart2, (uint8_t*)"IN \r\n", 13, 0xFFFF);

		  //TxHeader.StdId = 0x1ff;                 // CAN ID
		  TxHeader.StdId = p_target_can_id;                 // CAN ID
		  TxHeader.RTR = CAN_RTR_DATA;            // フレームタイプはデータフレーム
	      TxHeader.IDE = CAN_ID_STD;              // 標準ID(11ﾋﾞｯﾄ)
	      TxHeader.DLC = 8;                       // データ長は8バイトに
	      TxHeader.TransmitGlobalTime = DISABLE;  // ???

	      /*
	      TxData[0] = 0x3A;
	      TxData[1] = 0x98;
	      TxData[2] = 0x75;
	      TxData[3] = 0x30;
	      TxData[4] = 0xff;
	      TxData[5] = 0xff;
	      TxData[6] = 0x8A;
	      TxData[7] = 0xD0;
	      */

	      res = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, p_tx_data, &TxMailbox);
	      if(res != HAL_OK)
	      {
	    	  HAL_UART_Transmit( &huart2, (uint8_t*)"tx error   \r\n", 13, 0xFFFF);
	    	  Error_Handler();
	      }
	  }
	  else
	  {
		  HAL_UART_Transmit( &huart2, (uint8_t*)"box full \r\n", 13, 0xFFFF);
	  }

}


//---------------------------------------------------------------------------------------------------------------------
// @brief	: 速度指令値をPWMのコンペア値に変換する（GM6020用）
// @date	: 2021/04/29
//
// @param1[in]	: 速度指令値(-1.0 ~ 1.0)
// @return			: PWMのコンペア値
//---------------------------------------------------------------------------------------------------------------------
int ConvertSpeedControlValue2PwmCompareCount(float p_speed_control_value)
{
	// GM6020の正転逆転のPWMのコンペア値の境界値（未使用のものは記録用）
	//int ccw_min_compare_value  = PWM_TIMER_COUNTER_PERIOD * 0.054; // 1080us
	int ccw_max_compare_value = PWM_TIMER_COUNTER_PERIOD * 0.074; // 1480us
	int stop_compare_value        = PWM_TIMER_COUNTER_PERIOD * 0.075; // 1500us
	int cw_min_compare_value   = PWM_TIMER_COUNTER_PERIOD * 0.076; // 1520us
	//int cw_max_compare_value  = PWM_TIMER_COUNTER_PERIOD * 0.096; // 1920us

	// 速度指令値の絶対値がこの閾値以下なら，速度指令値をゼロとして扱う(float対策)
	// 0.01くらいで正転に個体差が出てくる
	float zero_threshold = 0.0001;
	float range_min2max_compare_value = PWM_TIMER_COUNTER_PERIOD * 0.020;

	int pwm_compare_count = 0;

	// 停止
	if(abs(p_speed_control_value) < zero_threshold)
	{
		pwm_compare_count = stop_compare_value;
	}
	// 正転
	else if(p_speed_control_value >= zero_threshold)
	{
		pwm_compare_count = cw_min_compare_value + int(range_min2max_compare_value * p_speed_control_value);
	}
	// 逆転
	else if(p_speed_control_value <= -zero_threshold)
	{
		pwm_compare_count = ccw_max_compare_value + int(range_min2max_compare_value * p_speed_control_value);
	}
	// 例外なし

	//char msgc[20] = "";
	//sprintf(msgc, " in %d   \r\n", pwm_compare_count);
	//HAL_UART_Transmit( &huart2, (uint8_t *)msgc, strlen(msgc) + 1, 0xFFFF);

	return pwm_compare_count;
}

//---------------------------------------------------------------------------------------------------------------------
// @brief : define.hで設定した値から，全方向で出せる車体の最大速度を求める
// @date	: 2021/04/29
//
// @return			: 車体の最大速度
//---------------------------------------------------------------------------------------------------------------------
float MaxRobotSpeed()
{
	// 車輪の最大速度[m/s]
	float max_wheel_vel = WHEEL_DIAMETER * M_PI * MAX_MOTOR_RPM / 60.0;
	// 車体の最大速度 [m/s]
	// 車輪の最大速度の√3/2倍から算出する(角度によって最小になる，車体の最大速度)
	float max_robot_vel = max_wheel_vel * (1.732051 / 2.0); // 1.732051 ≒ √3

	return max_robot_vel;
}


//---------------------------------------------------------------------------------------------------------------------
// @brief	: 正面ホイール側の回転速度を設定する
// @date	: 2021/04/29
//
// @param1[in]	: 速度指令値(-1.0 ~ 1.0)
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void SetPWMFront(float p_speed_command_value )
{
	// 回転方向が逆だから正転逆転を入れ替える
	//p_speed_command_value *= -1.0;
	// -1.0 ~ 1.0の範囲にクリッピングする
	p_speed_command_value = Clipping(p_speed_command_value, -1.0, 1.0);
	// 速度指令値からPWMのコンペアマッチのカウント値に変換する
	int p_pwm_compare_match_value = ConvertSpeedControlValue2PwmCompareCount(p_speed_command_value);
	// PWMのコンペアマッチのカウント値を設定する
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, p_pwm_compare_match_value);
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 左後ホイール側の回転速度を設定する
// @date	: 2021/04/29
//
// @param1[in]	: 速度指令値(-1.0 ~ 1.0)
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void SetPWMBackLeft(float p_speed_command_value)
{
	// 回転方向が逆だから正転逆転を入れ替える
	//p_speed_command_value *= -1.0;
	// -1.0 ~ 1.0の範囲にクリッピングする
	p_speed_command_value = Clipping(p_speed_command_value, -1.0, 1.0);
	// 速度指令値からPWMのコンペアマッチのカウント値に変換する
	int p_pwm_compare_match_value = ConvertSpeedControlValue2PwmCompareCount(p_speed_command_value);
	// PWMのコンペアマッチのカウント値を設定する
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, p_pwm_compare_match_value);
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 右後ホイール側の回転速度を設定する
// @date	: 2021/04/29
//
// @param1[in]	: 速度指令値(-1.0 ~ 1.0)
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void SetPWMBackRight(float p_speed_command_value)
{
	// 回転方向が逆だから正転逆転を入れ替える
	//p_speed_command_value *= -1.0;
	// -1.0 ~ 1.0の範囲にクリッピングする
	p_speed_command_value = Clipping(p_speed_command_value, -1.0, 1.0);
	// 速度指令値からPWMのコンペアマッチのカウント値に変換する
	int p_pwm_compare_match_value = ConvertSpeedControlValue2PwmCompareCount(p_speed_command_value);
	// PWMのコンペアマッチのカウント値を設定する
	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, p_pwm_compare_match_value);
}



//---------------------------------------------------------------------------------------------------------------------
// @brief	: 加速度を考慮した速度を計算する
// @date	: 2023/01/06
//
// @param1[in]	: 現在速度
// @param2[in]	: 目標速度
// @param3[in]	: 1ループで増加減する速度
// @return			: 加速度を考慮した速度
//---------------------------------------------------------------------------------------------------------------------
vector<float> CalcAcceleratedCmdValues(vector<float>p_accelerated_vel, vector<float> p_target_vel, float p_add_vel)
{




    if(p_accelerated_vel.size() != p_target_vel.size())
    {
    	HAL_UART_Transmit( &huart2, (uint8_t *)"p_accelerated_vel.size() != p_target_vel.size()", 100 , 0xFFFF);
        return p_accelerated_vel;
    }

    for(int i=0; i<p_target_vel.size(); i++)
    {
        float diff = p_target_vel[i] - p_accelerated_vel[i];
        float limited_add_vel = Clipping(diff, -p_add_vel, p_add_vel);

        p_accelerated_vel[i] += limited_add_vel;
    }


    return p_accelerated_vel;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
      char msg[40];
	  sprintf(msg, "Error Handler IN\r\n");
      HAL_UART_Transmit( &huart2, (uint8_t *)msg, strlen(msg) + 1, 0xFFFF);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
