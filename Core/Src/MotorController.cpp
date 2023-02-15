/*
 * MotorController.cpp
 *
 *  Created on: 2023/02/09
 *      Author: reo
 */

#include "MotorController.h"


MotorController::MotorController()
{
	// PWM
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	// HAL_Delay(10);

	// 安全のため初期デューティ比を0にする
	// __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0); // MD1
	// __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0); // MD2
	// __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0); // MD3

	// モータのPWMが指定の範囲外になったときにエラーになるから，立ち上げ時に0を送ってエラーリセットする
	// SetPWMFront(0);
	// SetPWMBackLeft(0);
	// SetPWMBackRight(0);



	m_accelerated_control_values.push_back(0);
	m_accelerated_control_values.push_back(0);
	m_accelerated_control_values.push_back(0);



}

MotorController::~MotorController()
{

}


//---------------------------------------------------------------------------------------------------------------------
// @brief	: 速度指令値から各車輪を駆動させる
// @date	: 2023/02/15
//
// @param1[in]	: 速度指令値X(-1.0 ~ 1.0)
// @param2[in]	: 速度指令値Y(-1.0 ~ 1.0)
// @param3[in]	: 速度指令値ω(-1.0 ~ 1.0)
// @return		: なし
//---------------------------------------------------------------------------------------------------------------------
void MotorController::SetSpeedControlValues(float p_speed_control_value_x, float p_speed_control_value_y, float p_speed_control_value_omega)
{

	vector<float> target_speed_control_values(3, 0);
	target_speed_control_values.push_back(p_speed_control_value_x     * TRANSLATION_RATIO);
	target_speed_control_values.push_back(p_speed_control_value_y     * TRANSLATION_RATIO);
	target_speed_control_values.push_back(p_speed_control_value_omega * ROTATION_RATIO);

	// ジョイスティックが円形か四角かで出力を変えると良い
	
    for(int i=0; i<m_accelerated_control_values.size(); i++)
	{
		// 念のためクリッピング
		target_speed_control_values[i]     = Clipping(target_speed_control_values[i], -1.0, 1.0);
		// 現在の速度と目標速度から加減速を考慮した速度を計算する
		m_accelerated_control_values[i] = CalcAcceleratedCmdValue(m_accelerated_control_values[i], target_speed_control_values[i], ACCELELATED_ADD_VEL);
	}

	// 車体速度から車輪速度を計算する
	vector<float> wheel_vels;
	wheel_vels = ConvertBodyVel2WheelVels(m_accelerated_control_values[0], m_accelerated_control_values[1], m_accelerated_control_values[2]);

	// PWM信号を送る
	SetPWMFront(    wheel_vels[0]);
	SetPWMBackLeft( wheel_vels[1]);
	SetPWMBackRight(wheel_vels[2]);
}


//---------------------------------------------------------------------------------------------------------------------
// @brief	: 加速度を考慮した速度を計算する
// @date	: 2023/02/15
//
// @param1[in]	: 現在速度
// @param2[in]	: 目標速度
// @param3[in]	: 1ループで増加減する速度
// @return		: 加速度を考慮した速度
//---------------------------------------------------------------------------------------------------------------------
float MotorController::CalcAcceleratedCmdValue(float p_accelerated_vel, float p_target_vel, float p_add_vel)
{
	// 現在の速度と目標速度から加減速を考慮した速度を計算する
    float diff = p_target_vel - p_accelerated_vel;
    float limited_add_vel = Clipping(diff, -p_add_vel, p_add_vel);
    p_accelerated_vel += limited_add_vel;

    return p_accelerated_vel;
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 車体速度からホイール速度へ変換する（運動学）
// @date	: 2021/02/11
//
// @param1[in]	: 車体速度x
// @param2[in]	: 車体速度y
// @param3[in]	: 車体角速度 [rad]
// @param4[out]: 車輪速度前
// @param5[out]: 車輪速度左後
// @param6[out]: 車輪速度右後
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
vector<float> MotorController::ConvertBodyVel2WheelVels(float p_vx, float p_vy, float p_omega)
{
	vector<float> wheel_vels(3, 0);

	// < 車輪Frontがx軸に正 >
    wheel_vels[0] =           0 * p_vx  +  1.0 * p_vy  +  p_omega; // flont
    wheel_vels[1] =  -0.8660254 * p_vx  -  0.5 * p_vy  +  p_omega; // left back
    wheel_vels[2] =   0.8660254 * p_vx  -  0.5 * p_vy  +  p_omega; // right back
	return wheel_vels;
}


//---------------------------------------------------------------------------------------------------------------------
// @brief	: 速度指令値をPWMのコンペア値に変換する（GM6020用）
// @date	: 2021/04/29
//
// @param1[in]	: 速度指令値(-1.0 ~ 1.0)
// @return			: PWMのコンペア値
//---------------------------------------------------------------------------------------------------------------------
int MotorController::ConvertSpeedControlValue2PwmCompareCount(float p_speed_control_value)
{
	// GM6020の正転逆転のPWMのコンペア値の境界値（未使用のものは記録用）
	//int ccw_min_compare_value  = PWM_TIMER_COUNTER_PERIOD * 0.054; // 1080us
	int ccw_max_compare_value = PWM_TIMER_COUNTER_PERIOD * 0.074; // 1480us
	int stop_compare_value    = PWM_TIMER_COUNTER_PERIOD * 0.075; // 1500us
	int cw_min_compare_value  = PWM_TIMER_COUNTER_PERIOD * 0.076; // 1520us
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
// @brief	: 正面ホイール側の回転速度を設定する
// @date	: 2021/04/29
//
// @param1[in]	: 速度指令値(-1.0 ~ 1.0)
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void MotorController::SetPWMFront(float p_speed_control_value )
{
	// 回転方向が逆だから正転逆転を入れ替える
	//p_speed_control_value *= -1.0;
	// -1.0 ~ 1.0の範囲にクリッピングする
	p_speed_control_value = Clipping(p_speed_control_value, -1.0, 1.0);
	// 速度指令値からPWMのコンペアマッチのカウント値に変換する
	int p_pwm_compare_match_value = ConvertSpeedControlValue2PwmCompareCount(p_speed_control_value);
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
void MotorController::SetPWMBackLeft(float p_speed_control_value)
{
	// 回転方向が逆だから正転逆転を入れ替える
	//p_speed_control_value *= -1.0;
	// -1.0 ~ 1.0の範囲にクリッピングする
	p_speed_control_value = Clipping(p_speed_control_value, -1.0, 1.0);
	// 速度指令値からPWMのコンペアマッチのカウント値に変換する
	int p_pwm_compare_match_value = ConvertSpeedControlValue2PwmCompareCount(p_speed_control_value);
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
void MotorController::SetPWMBackRight(float p_speed_control_value)
{
	// 回転方向が逆だから正転逆転を入れ替える
	//p_speed_control_value *= -1.0;
	// -1.0 ~ 1.0の範囲にクリッピングする
	p_speed_control_value = Clipping(p_speed_control_value, -1.0, 1.0);
	// 速度指令値からPWMのコンペアマッチのカウント値に変換する
	int p_pwm_compare_match_value = ConvertSpeedControlValue2PwmCompareCount(p_speed_control_value);
	// PWMのコンペアマッチのカウント値を設定する
	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, p_pwm_compare_match_value);
}






//---------------------------------------------------------------------------------------------------------------------
// @brief	: モータをキャリブレーションする（使わなそうだからコメントアウト）
// @date	: 2023/01/06
//
// @param1[in]	: 現在速度
// @param2[in]	: 目標速度
// @param3[in]	: 1ループで増加減する速度
// @return		: 加速度を考慮した速度
//---------------------------------------------------------------------------------------------------------------------
void MotorController::CalibrateMotors()
{
	/*
	// GM6020キャリブレーション
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==GPIO_PIN_RESET){
		//HAL_UART_Transmit( &huart2, (uint8_t *)"ON\n", strlen("ON\n") + 1, 0xFFFF);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 3000); // 起動時
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 3000); // 起動時
		__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 3000); // 起動時
	}
	else
	{
		//HAL_UART_Transmit( &huart2, (uint8_t *)"OFF\n", strlen("OFF\n") + 1, 0xFFFF);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 6000);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 6000);
		__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 6000);
	}
	*/
}


//---------------------------------------------------------------------------------------------------------------------
// @brief	: 指定の範囲外だった場合にその範囲内に収める
// @date	: 2021/02/11
//
// @param1[in]	: クリップする値
// @param1[in]	: 範囲の下限
// @param1[in]	: 範囲の上限
// @return		: 指定の範囲内にクリップされた値
//---------------------------------------------------------------------------------------------------------------------
float MotorController::Clipping(float val, float lower_bound, float upper_bound)
{
    if(val < lower_bound) {
        val = lower_bound;
    } else if(upper_bound < val) {
        val = upper_bound;
    }
    return val;
}

