/*
 * Encoder.cpp
 *
 *  Created on: Feb 10, 2021
 *      Author: renew
 */

/*
//template

//---------------------------------------------------------------------------------------------------------------------
// @brief	:
// @date	: 2021/02/11
//
// @param1[in]	: []
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void Encoder::()
{

}

*/


#include "Encoder.h"
#include "CommonMath.h"
#include "usart.h"
#include "i2c.h"


#include "stdio.h"  // Encoder
#include "string.h" // Encoder


uint32_t readed_can_id;
uint32_t readed_can_dlc;
uint8_t id517_readed_can_data[8];
uint8_t id518_readed_can_data[8];
uint8_t id519_readed_can_data[8];
int cnt = 0;
int id_1_cnt = 0;
int id_2_cnt = 0;
int id_3_cnt = 0;

int pre_absoluted_enc_value[3];



//---------------------------------------------------------------------------------------------------------------------
// @brief	: HALのCANコールバック関数
// @date	: 2021/04/30
//
// @param1[out]	: CANハンドル
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	cnt++;
	if(cnt%100 != 0){return;}

    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
    	readed_can_id = (RxHeader.IDE == CAN_ID_STD)? RxHeader.StdId : RxHeader.ExtId;     // ID
    	readed_can_dlc = RxHeader.DLC;                                                     // DLC

        if(readed_can_id == 517)
        {
			id517_readed_can_data[0] = RxData[0]; // 格納する
			id517_readed_can_data[1] = RxData[1];
			id517_readed_can_data[2] = RxData[2];
			id517_readed_can_data[3] = RxData[3];
			id517_readed_can_data[4] = RxData[4];
			id517_readed_can_data[5] = RxData[5];
			id517_readed_can_data[6] = RxData[6];
			id517_readed_can_data[7] = RxData[7];
        }
        else if(readed_can_id == 518)
        {
			id518_readed_can_data[0] = RxData[0]; // 格納する
			id518_readed_can_data[1] = RxData[1];
			id518_readed_can_data[2] = RxData[2];
			id518_readed_can_data[3] = RxData[3];
			id518_readed_can_data[4] = RxData[4];
			id518_readed_can_data[5] = RxData[5];
			id518_readed_can_data[6] = RxData[6];
			id518_readed_can_data[7] = RxData[7];
        }
        else if(readed_can_id == 520)
        {
			id519_readed_can_data[0] = RxData[0]; // 格納する
			id519_readed_can_data[1] = RxData[1];
			id519_readed_can_data[2] = RxData[2];
			id519_readed_can_data[3] = RxData[3];
			id519_readed_can_data[4] = RxData[4];
			id519_readed_can_data[5] = RxData[5];
			id519_readed_can_data[6] = RxData[6];
			id519_readed_can_data[7] = RxData[7];
        }


        if(readed_can_id == 517){ id_1_cnt++;}
        else if(readed_can_id == 518){ id_2_cnt++;}
        else if(readed_can_id == 520){ id_3_cnt++;}
    }
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: エンコーダの初期設定を行う
// @date	: 2021/05/1
//
// @param1[]	: なし
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------

void InitEncoder()
{
	// CANが先に来ないとpre_absoluted_enc_valueの値が先に来てしまう。そのためリセットしても自己位置が0にならない。
	HAL_Delay(100);

	// エンコーダの前の値を格納しておく
	int absoluted_enc_value[3];
	GetAbsolutedEncCount(&absoluted_enc_value[0], &absoluted_enc_value[1], &absoluted_enc_value[2]);

	// 差分をとるために前の値を格納する
	pre_absoluted_enc_value[0] = absoluted_enc_value[0];
	pre_absoluted_enc_value[1] = absoluted_enc_value[1];
	pre_absoluted_enc_value[2] = absoluted_enc_value[2];

	float wheel_movements[3];
	GetWheelMovements(&wheel_movements[0], &wheel_movements[1], &wheel_movements[2]);

}



//---------------------------------------------------------------------------------------------------------------------
// @brief	: エンコーダの現在値を取得する
// @date	: 2021/04/30
//
// @param1[out]	: 前車輪のエンコーダ値
// @param2[out]	: 左車輪のエンコーダ値
// @param3[out]	: 右車輪のエンコーダ値
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void GetAbsolutedEncCount(int *p_front_absoluted_enc_count, int *p_left_absoluted_enc_count, int *p_right_absoluted_enc_count)
{
	int absoluted_enc_count[3];
	absoluted_enc_count[0] = (id517_readed_can_data[0] << 8) | id517_readed_can_data[1];
	absoluted_enc_count[1] = (id518_readed_can_data[0] << 8) | id518_readed_can_data[1];
	absoluted_enc_count[2] = (id519_readed_can_data[0] << 8) | id519_readed_can_data[1];

	// 正転でカウントが減少していくから，増加するように修正する
	*p_front_absoluted_enc_count = ABSOLUTE_ENC_MAX_VALU - absoluted_enc_count[0];
	*p_left_absoluted_enc_count   = ABSOLUTE_ENC_MAX_VALU - absoluted_enc_count[1];
	*p_right_absoluted_enc_count = ABSOLUTE_ENC_MAX_VALU - absoluted_enc_count[2];

	//char msg[50];
    //sprintf(msg, " %7d %7d %7d \r\n", p_absoluted_enc_count[0], p_absoluted_enc_count[1], p_absoluted_enc_count[2]);
    //HAL_UART_Transmit( &huart2, msg, strlen(msg) + 1, 0xFFFF);
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 車輪エンコーダ値の変化量を取得する
// @date	: 2021/04/30
//
// @param1[out]	: 前車輪のエンコーダ値の変化量
// @param2[out]	: 左車輪のエンコーダ値の変化量
// @param3[out]	: 右車輪のエンコーダ値の変化量
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void GetDiffEncCount(int *p_front_diff_enc_count, int *p_left_diff_enc_count, int *p_right_diff_enc_count)
{
	int absoluted_enc_value[3];
	GetAbsolutedEncCount(&absoluted_enc_value[0], &absoluted_enc_value[1], &absoluted_enc_value[2]);

	// 	差分を求める
	int diff_enc_value[3];
	diff_enc_value[0] = absoluted_enc_value[0] - pre_absoluted_enc_value[0];
	diff_enc_value[1] = absoluted_enc_value[1] - pre_absoluted_enc_value[1];
	diff_enc_value[2] = absoluted_enc_value[2] - pre_absoluted_enc_value[2];

	// 周またぎした場合は修正する
	int straddle_threshold =  (ABSOLUTE_ENC_MAX_VALU + 1) / 2;
	diff_enc_value[0] = ModifyStraddleEncValue(diff_enc_value[0], straddle_threshold);
	diff_enc_value[1] = ModifyStraddleEncValue(diff_enc_value[1], straddle_threshold);
	diff_enc_value[2] = ModifyStraddleEncValue(diff_enc_value[2], straddle_threshold);

	// 引数ポインタに格納する
	*p_front_diff_enc_count = diff_enc_value[0];
	*p_left_diff_enc_count   = diff_enc_value[1];
	*p_right_diff_enc_count = diff_enc_value[2];

	// 差分をとるために前の値を格納する
	pre_absoluted_enc_value[0] = absoluted_enc_value[0];
	pre_absoluted_enc_value[1] = absoluted_enc_value[1];
	pre_absoluted_enc_value[2] = absoluted_enc_value[2];
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 車輪の移動量を取得する
// @date	: 2021/05/1
//
// @param1[out]	: 前車輪の移動量 [m]
// @param2[out]	: 左車輪の移動量 [m]
// @param3[out]	: 右車輪の移動量 [m]
// @return		: なし
//---------------------------------------------------------------------------------------------------------------------
void GetWheelMovements(float *p_front_wheel_movement, float *p_left_wheel_movement, float *p_right_wheel_movement)
{
	int diff_enc_cnt[3];
    GetDiffEncCount(&diff_enc_cnt[0], &diff_enc_cnt[1], &diff_enc_cnt[2]);

    // 各車輪の移動量 [m]
    float wheel_movements[3];
    wheel_movements[0] = ConvertEncoderDiff2Movement(diff_enc_cnt[0]);
    wheel_movements[1] = ConvertEncoderDiff2Movement(diff_enc_cnt[1]);
    wheel_movements[2] = ConvertEncoderDiff2Movement(diff_enc_cnt[2]);

    *p_front_wheel_movement = wheel_movements[0];
    *p_left_wheel_movement   = wheel_movements[1];
    *p_right_wheel_movement = wheel_movements[2];
}



//---------------------------------------------------------------------------------------------------------------------
// @brief	: 周跨ぎしていた場合はエンコーダ値の変化量を修正する
// @date	: 2021/04/30
//
// @param1[in]	: エンコーダ値の変化量
// @param2[in]	: 周跨ぎ判定の閾値 [count]
// @return			: 修正したエンコーダ値の変化量
//---------------------------------------------------------------------------------------------------------------------
int ModifyStraddleEncValue(int p_diff_enc_value, int p_straddle_threshold)
{
	// 正転方向
	if(p_diff_enc_value < -p_straddle_threshold)
	{
		p_diff_enc_value += ABSOLUTE_ENC_MAX_VALU + 1;
	}
	// 逆転方向
	if(p_diff_enc_value > p_straddle_threshold)
	{
		p_diff_enc_value -= ABSOLUTE_ENC_MAX_VALU + 1;
	}
	return p_diff_enc_value;
}



//---------------------------------------------------------------------------------------------------------------------
// @brief	: エンコーダ値の変化量から移動量を求める
// @date	: 2021/04/30
//
// @param1[out]	: エンコーダ値の変化量 [count]
// @return				: 移動量 [m]
//---------------------------------------------------------------------------------------------------------------------
float ConvertEncoderDiff2Movement(int p_diff_enc_value)
{
	// 何周したか [周]
	float lap_value = (float)p_diff_enc_value / (float)ABSOLUTE_ENC_MAX_VALU;
	// 車輪の周囲長 [m]
	float wheel_perimeter = M_PI * WHEEL_DIAMETER;
	// 移動量 [m]
	float movement = lap_value * wheel_perimeter;

	return movement;
	//char msg[50];
    //sprintf(msg, " %7d %7d %7d \r\n", p_absoluted_enc_count[0], p_absoluted_enc_count[1], p_absoluted_enc_count[2]);
    //HAL_UART_Transmit( &huart2, msg, strlen(msg) + 1, 0xFFFF);
}








Encoder::Encoder()
{
	// 自己位置初期化
	m_pos.x = 0;
	m_pos.y = 0;
	m_pos.angle = 0;

	// エンコーダ4逓倍
	m_encoder_multiplication = 4.0;

	// メモリ領域を確保する
	for(int i=0; i<ENCODER_QTY; i++)
	{
		m_enc_counts.push_back(0);
		m_wheel_velocities.push_back(0);
		m_wheel_angular_velocities.push_back(0);
		m_angle_diffs.push_back(0);
		m_wheel_dist_diffs.push_back(0);

	}

	// エンコーダの読み取りを開始する
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);

	// BNO055
	unsigned char address = 0x28; // 0x28; //0x50; //0x29;
	//BNO055 _bno055(hi2c1,address);
	bno055.Init(hi2c1,address);
	//  BNO055の初期角度を取得する
	m_bno_init_rad = bno055.GetYawRadian();
	m_pre_bno_rad =  bno055.GetYawRadian();
	m_bno_rad = 0;


	// 位置取得用エンコーダ1周あたりのパス数
	m_position_enc_ppr = POSITION_ENC_PPR;
	// エンコーダカウンタの最大値
	m_encoder_count_max_value = ENCODER_COUNT_BUFF_MAX_VALUE;

}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 駆動軸用エンコーダの分解能を設定する
// @date	: 2021/02/11
//
// @param1[in]	: 設定したい分解能
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void Encoder::SetDriveEncoderPPR(int p_ppr)
{
	m_drive_encoder_per_pulse_rotation = p_ppr;
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: ギア比を設定する
// @date	: 2021/02/11
//
// @param1[in]	: 設定したいギア比
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void Encoder::SetGearRatio(float p_gear_ratio)
{
	m_gear_ratio = p_gear_ratio;
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 駆動用ホイールの直径を設定する
// @date	: 2021/02/11
//
// @param1[in]	: 設定したいホイールの直径[m]
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void Encoder::SetDriveWheelDiameter(float p_drive_wheel_diameter)
{
	m_drive_wheel_diameter = p_drive_wheel_diameter;
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 位置推定用ホイールの直径を設定する
// @date	: 2021/02/11
//
// @param1[in]	: 設定したいホイールの直径[m]
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void Encoder::SetPositionWheelDiameter(float p_position_wheel_diameter)
{
	m_position_wheel_diameter = p_position_wheel_diameter;
}


//---------------------------------------------------------------------------------------------------------------------
// @brief	: ホイール速度を取得する
// @date	: 2021/02/11
//
// @param1[in]	: なし
// @return			: ホイール速度[m/s]
//---------------------------------------------------------------------------------------------------------------------
vector<float> Encoder::GetWheelVelocities()
{
	return m_wheel_velocities;
}


//---------------------------------------------------------------------------------------------------------------------
// @brief	: 位置姿勢を取得する
// @date	: 2021/02/11
//
// @param1[in]	: なし
// @return			: 位置姿勢 < x[mm], y[mm], angle[rad] >
//---------------------------------------------------------------------------------------------------------------------
struct_posture_t Encoder::GetPositions()
{
	return m_pos;
}



//---------------------------------------------------------------------------------------------------------------------
// @brief	: 位置計測用のエンコーダ値の変化量を取得する(ゼロリセットも含む)
// @date	: 2021/07/26
//
// @param1[out]	: X位置計測用エンコーダの変化量
// @param2[out]	: Y位置計測用エンコーダの変化量
// @retrun		:  なし
//---------------------------------------------------------------------------------------------------------------------
void Encoder::GetPositionEncCounts(int *p_enc_cnt_x, int *p_enc_cnt_y)
{
	// xが基板の上側，yが基板の下側のコネクタ
	int enc_cnt_x = TIM3 -> CNT; // Xエンコーダ値の値を取得する
	TIM3 -> CNT = 0; // リセット
	int enc_cnt_y = TIM1 -> CNT; // Yエンコーダ値の値を取得する
	TIM1 -> CNT = 0;

	// 逆回転だった場合はマイナスに修正する
	if(enc_cnt_x > ENCODER_COUNT_BUFF_MAX_VALUE/2)
	{
		enc_cnt_x -= ENCODER_COUNT_BUFF_MAX_VALUE + 1;
	}
	if(enc_cnt_y > ENCODER_COUNT_BUFF_MAX_VALUE/2)
	{
		enc_cnt_y -= ENCODER_COUNT_BUFF_MAX_VALUE + 1;
	}

	*p_enc_cnt_x = enc_cnt_x;
	*p_enc_cnt_y = enc_cnt_y;
}
//---------------------------------------------------------------------------------------------------------------------
// @brief	: 位置取得用のエンコーダ値の変化量から移動量を求める
// @date	: 2021/04/30
//
// @param1[out]	: エンコーダ値の変化量 [count]
// @return				: 移動量 [m]
//---------------------------------------------------------------------------------------------------------------------
float Encoder::ConvertPositionEncDiff2Movement(int p_diff_enc_value)
{
	// 何周したか [周]
	float lap_value = (float)p_diff_enc_value / (float)m_position_enc_ppr;
	// 車輪の周囲長 [m]
	float wheel_perimeter = M_PI * ENCODER_WHEEL_DIAMETER;
	// 移動量 [m]
	float movement = lap_value * wheel_perimeter;

	return movement;
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	:ロボット座標系の移動量を取得する
// @date	: 2021/05/1
//
// @param1[out]	: 前車輪の移動量 [m]
// @param2[out]	: 左車輪の移動量 [m]
// @param3[out]	: 右車輪の移動量 [m]
// @return		: なし
//---------------------------------------------------------------------------------------------------------------------
void Encoder::GetMovements(float *p_movement_x, float *p_movement_y)
{
	int enc_cnt_x;
	int enc_cnt_y;
	// エンコーダのカウント値（変化量）を取得する
	GetPositionEncCounts(&enc_cnt_x, &enc_cnt_y);
	// カウント値[cnt]から移動量[m]に変換する
	float movement_x = ConvertPositionEncDiff2Movement(enc_cnt_x);
	float movement_y = ConvertPositionEncDiff2Movement(enc_cnt_y);
	// 移動方向逆だから負の値を掛ける
	//movement_x *= -1.0;
	//movement_y *= -1.0;

	*p_movement_x = movement_x;
	*p_movement_y = movement_y;
}




//---------------------------------------------------------------------------------------------------------------------
// @brief	: エンコーダのカウント値を取得して内部情報を更新する
// @date	: 2021/02/11
//
// @param1[in]	: なし
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void Encoder::Update()
{
	// 元駆動輪エンコーダ版

	/*
	float wheel_movements[3];
	// 車輪の移動量を取得する
	GetWheelMovements(&wheel_movements[0], &wheel_movements[1], &wheel_movements[2]); // [m]
	// 車輪の速度を取得する
	m_wheel_velocities[0] = wheel_movements[0] / UPDATE_TIME_INTERVAL; // [m/s]
	m_wheel_velocities[1] = wheel_movements[1] / UPDATE_TIME_INTERVAL; // [m/s]
	m_wheel_velocities[2] = wheel_movements[2] / UPDATE_TIME_INTERVAL; // [m/s]


	// BNO055
	//  BNO055の値を仮でこの場所で取得する
	//float bno_yaw_rad = bno055.GetYawRadian();

	float now_bno_rad = bno055.GetYawRadian();
	float diff_rad = now_bno_rad - m_pre_bno_rad;
	if(diff_rad <= -M_PI/2.0)
	{
		diff_rad += 2*M_PI;
	}
	else if(diff_rad >= M_PI/2.0)
	{
		diff_rad -= 2*M_PI;
	}
	m_bno_rad += diff_rad;
	//bno_yaw_rad = bno_yaw_rad;


	//float diff = bno055.GetYawRadian() - m_pre_bno_rad;
	//if(abs(diff) > 360 )
	//{
	//	bno_yaw_rad =
	//}
	m_pre_bno_rad = now_bno_rad;

	// 位置推定
	// ロボット座標系
	struct_posture_t diff_local_pos; // x[m], y[m], angle[rad]
	// マップ座標系
	struct_posture_t diff_global_pos; // x[m], y[m], angle[rad]
	// 車輪の移動量から車体の移動量に変換する
	ConvertWheelVel2BodyVel(wheel_movements[0], wheel_movements[1], wheel_movements[2], &diff_local_pos.x, &diff_local_pos.y, &diff_local_pos.angle);

	// a現状だと自己位置精度が悪いので角度をジャイロの値を使うことで代用する。

	// 移動量をロボット座標系からマップ座標系に変換する
	//Rotation(diff_local_pos.x, diff_local_pos.y, m_pos.angle + diff_local_pos.angle / 2, &diff_global_pos.x, &diff_global_pos.y);
	//Rotation(diff_local_pos.x, diff_local_pos.y, bno_yaw_rad, &diff_global_pos.x, &diff_global_pos.y);
	Rotation(diff_local_pos.x, diff_local_pos.y, m_bno_rad, &diff_global_pos.x, &diff_global_pos.y);
	// 角度変化量は変わらないからそのまま
	diff_global_pos.angle = diff_local_pos.angle;
	// 移動量を加算する
	m_pos.x += diff_global_pos.x;
	m_pos.y += diff_global_pos.y;
	//m_pos.angle = bno_yaw_rad;
	m_pos.angle = m_bno_rad;
	//m_pos.angle += diff_global_pos.angle;
	*/


	// 自己位置取得用エンコーダ版

	// < BNO055 >
	// yaw角を取得する [rad]
	float now_bno_rad = bno055.GetYawRadian();
	// 差分を取得する
	float diff_rad = now_bno_rad - m_pre_bno_rad;
	// 絶対角度になってるから回転量に変換する
	if(diff_rad <= -M_PI/2.0)
	{
		diff_rad += 2*M_PI;
	}
	else if(diff_rad >= M_PI/2.0)
	{
		diff_rad -= 2*M_PI;
	}

	// < 位置取得用エンコーダ >
	float diff_local_x; // ロボット座標系での移動量x[m]
	float diff_local_y; // ロボット座標系での移動量y[m]
	// ロボット座標系での移動量を取得する
	GetMovements(&diff_local_x, &diff_local_y);
	// 移動量をロボット座標系からグローバル座標系へ変換する
	float diff_global_x;
	float diff_global_y;
	Rotation(diff_local_x, diff_local_y, m_bno_rad + (diff_rad/2.0), &diff_global_x, &diff_global_y);
	//Rotation(diff_local_x, diff_local_y, m_bno_rad + diff_rad, &diff_global_x, &diff_global_y);


	// 回転量を加算する
	m_bno_rad += diff_rad;
	// 前の値を格納しておく
	m_pre_bno_rad = now_bno_rad;

	// 計算した値を格納する
	m_pos.x += diff_global_x; // [m]
	m_pos.y += diff_global_y; // [m]
	m_pos.angle = m_bno_rad; // [rad]

	//char sprintbuff[64] = "";
	//unsigned char uartbuff[64];
	//sprintf(sprintbuff,"%10f  %10f  %10f\n", m_pos.x, m_pos.y, m_bno_rad);
	//memcpy(uartbuff,sprintbuff,sizeof(sprintbuff));
	//HAL_UART_Transmit(&huart2,uartbuff,sizeof(uartbuff),1000);



	// ジャイロ角度表示用
	//char sprintbuff[64] = "";
	//unsigned char uartbuff[64];
	//sprintf(sprintbuff,"%7.3f\n", (diff_rad) * 180 / 3.14159265);
	//memcpy(uartbuff,sprintbuff,sizeof(sprintbuff));
	//HAL_UART_Transmit(&huart2,uartbuff,sizeof(uartbuff),1000);




	//char msg[40];
	//sprintf(msg, "%5d %5d %5d %5d %5d \r\n", m_enc_counts[0], m_enc_counts[1], m_enc_counts[2], m_enc_counts[3], m_enc_counts[4]);
	//sprintf(msg, "%5.5f %5.5f %5.5f %5.5f %5.5f \r\n", m_wheel_dist_diffs[0], m_wheel_dist_diffs[1], m_wheel_dist_diffs[2], m_wheel_dist_diffs[3], m_wheel_dist_diffs[4]);
	//sprintf(msg, "@@ %f %f %f %f %f \r\n", m_angle_diffs[0], m_angle_diffs[1], m_angle_diffs[2], m_angle_diffs[3], m_angle_diffs[4]);
	//sprintf(msg, "%d %f %f %f     \r\n", m_enc_counts[0], m_angle_diffs[0], m_wheel_dist_diffs[0], ang);
	//sprintf(msg, "%5.5f %5.5f %5.5f \r\n", m_wheel_velocities[0], m_wheel_velocities[1], m_wheel_velocities[2]);
	//sprintf(msg, "%5.5f %5.5f %5.5f \r\n", diff_pos.x, diff_pos.y, diff_pos.angle);
	//sprintf(msg, "%5.5f %5.5f %5.5f \r\n", m_pos.x, m_pos.y, m_pos.angle);

	//sprintf(msg, "%5.5f %5.5f %5.5f \r\n", wheel_movements[0], wheel_movements[1], wheel_movements[2]);

	//HAL_UART_Transmit( &huart2, (uint8_t *)msg, strlen(msg) + 1, 0xFFFF);

	//char msg[] = "Hello!\r\n";
	//HAL_UART_Transmit( &huart2, (uint8_t *)msg, strlen(msg) + 1, 0xFFFF);
}
