/*
 * Define.h
 *
 *  Created on: 2021/02/11
 *      Author: renew
 */

#ifndef INC_DEFINE_H_
#define INC_DEFINE_H_

// PWM設定
// 周波数 42MHz / (3 * 1000) = 14kHz
// 分解能 0 ~ 1000

#define M_PI 3.14159265358979323846
// 制御周期
#define UPDATE_TIME_INTERVAL 0.02 //0.01 // [sec]
#define SERIAL_RECIEVE_ARRAY_SIZE 20
// モータの数
#define MOTOR_QTY 3
// エンコーダの数
#define ENCODER_QTY 5
// エンコーダのカウンタの最大値
#define ENCODER_COUNT_MAX_SIZE 65536

// PWMのカウンターピリオド（カウント最大値）
#define PWM_TIMER_COUNTER_PERIOD 59999//60000
// シリアルDMAのリングバッファ容量
#define SERIAL_RECEIVE_BUFFER_SIZE 100

// 未使用（記録用）
// モータの最大回転速度[rpm]
#define MAX_MOTOR_RPM 100
// 車輪直径[m]
#define WHEEL_DIAMETER 0.122682 // 0.12 // この誤差なに？全部同じようにでてるんだけど
// アブソリュートエンコーダの最大値[count]
#define ABSOLUTE_ENC_MAX_VALU 8191

// 位置取得用エンコーダの1回転でのパルス数 [pulse]
#define POSITION_ENC_PPR 4000 // 1000を4逓倍してる
// エンコーダ取得用バッファの最大カウント数[count]
#define ENCODER_COUNT_BUFF_MAX_VALUE 65535
// 位置取得用エンコーダの車輪直径[m]
#define ENCODER_WHEEL_DIAMETER 0.0475 //0.05931// テンションでの沈み考慮値．理論値は0.06


#define FRONT 0
#define LEFT 1
#define RIGHT 2







typedef struct{
	float x; // [mm]
	float y; // [mm]
	float angle; // [rad]
} struct_posture_t;



#endif /* INC_DEFINE_H_ */
