/*
 * Encoder.h
 *
 *  Created on: Feb 10, 2021
 *      Author: renew
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_


#include "tim.h"
#include <vector>
#include "Define.h"

#include "BNO055.h"


using namespace std;

void InitEncoder();
// @brief	: エンコーダの現在値を取得する
void GetAbsolutedEncCount(int *p_front_absoluted_enc_count, int *p_left_absoluted_enc_count, int *p_right_absoluted_enc_count);
// @brief	: エンコーダ値の変化量を取得する
void GetDiffEncCount(int *p_front_diff_enc_count, int *p_left_diff_enc_count, int *p_right_diff_enc_count);
// @brief	: 車輪の移動量を取得する
void GetWheelMovements(float *p_front_wheel_movement, float *p_left_wheel_movement, float *p_right_wheel_movement);
// @brief	: 周跨ぎしていた場合はエンコーダ値の変化量を修正する
int ModifyStraddleEncValue(int p_diff_enc_value, int p_straddle_threshold);
// @brief	: エンコーダ値の変化量から移動量を求める
float ConvertEncoderDiff2Movement(int p_diff_enc_value);


// エンコーダカウンタ

class Encoder
{
public:
	Encoder();
	// データ取得＆情報更新を行う
	void Update();
	// 駆動軸用エンコーダの分解能を設定する
	void SetDriveEncoderPPR(int p_ppr);
	// ギア比を設定する
	void SetGearRatio(float p_gear_ratio);
	// 駆動用ホイールの直径を設定する
	void SetDriveWheelDiameter(float p_drive_wheel_diameter);
	// 位置推定用ホイールの直径を設定する
	void SetPositionWheelDiameter(float p_position_wheel_diameter);
	// ホイール速度を取得する
	vector<float> GetWheelVelocities();
	// 位置姿勢を取得する
	struct_posture_t GetPositions();


	// 追加

	//  <<位置計測用エンコーダ>>
	// @brief	: 位置計測用のエンコーダ値の変化量を取得する(ゼロリセットも含む)
	void GetPositionEncCounts(int *p_enc_cnt_x, int *p_enc_cnt_y);
	// @brief	: 位置取得用のエンコーダ値の変化量から移動量を求める
	float ConvertPositionEncDiff2Movement(int p_diff_enc_value);
	// @brief	:ロボット座標系の移動量を取得する
	void GetMovements(float *p_movement_x, float *p_movement_y);

private:
	float ang;
	// 逓倍
	float m_encoder_multiplication;
	// 取得したエンコーダのカウント値
	vector<int> m_enc_counts;
	// 角度変化量
	vector<float> m_angle_diffs;
	// ホイールの移動距離
	vector<float> m_wheel_dist_diffs; // [m]
	// ホイールの回転速度
	vector<float> m_wheel_velocities; // [m/s]
	// ホイールの回転角速度
	vector<float> m_wheel_angular_velocities; // [rad/s]
	// ギア比 (1.0以下)
	float m_gear_ratio;
	// 駆動用エンコーダの分解能
	int m_drive_encoder_per_pulse_rotation; // [PPR]
	// 位置推定用エンコーダの1回転あたりのパルス数
	int m_position_enc_ppr; // [PPR]
	// 駆動用ホイール直径
	float m_drive_wheel_diameter; // [m]
	// 位置推定用ホイール直径
	float m_position_wheel_diameter; // [m]
	// 位置姿勢
	struct_posture_t m_pos; // < x[mm], y[mm], angle[rad] >
	// 初期角度
	float m_bno_init_rad; // [rad]
	float m_bno_rad;
	float m_pre_bno_rad; // [rad]
	BNO055 bno055;

	// 追加

	// エンコーダカウンタの最大値
	int m_encoder_count_max_value; // [count]



};



#endif /* INC_ENCODER_H_ */
