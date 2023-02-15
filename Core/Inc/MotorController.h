/*
 * MotorController.h
 *
 *  Created on: 2023/02/09
 *      Author: reo
 */

#ifndef INC_MOTORCONTROLLER_H_
#define INC_MOTORCONTROLLER_H_

#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


#include "CommonMath.h"
#include "stdio.h"
#include "string.h"
#include <math.h>

// 1周期で増減する速度
#define ACCELELATED_ADD_VEL 0.03
#define TRANSLATION_RATIO 0.2
#define ROTATION_RATIO 0.2

// PWMのカウンターピリオド（カウント最大値）
#define PWM_TIMER_COUNTER_PERIOD 59999//60000




class MotorController
{
public:
	MotorController();
	~MotorController();
	// 速度指令値から各車輪を駆動させる
	void SetSpeedControlValues(float, float, float);
	// 速度指令値をPWMのコンペア値に変換する
	int ConvertSpeedControlValue2PwmCompareCount(float);
	// 正面ホイール側の回転速度を設定する
	void SetPWMFront(float);
	// 左後ホイール側の回転速度を設定する
	void SetPWMBackLeft(float);
	// 右後ホイール側の回転速度を設定する
	void SetPWMBackRight(float);

	// データ取得＆情報更新を行う
	void Update();
	// 駆動軸用エンコーダの分解能を設定する
	void SetDriveEncoderPPR(int);
	// 加速度を考慮した速度を計算する
	float CalcAcceleratedCmdValue(float, float, float);
	// 加速度を考慮した速度を計算する
	vector<float> CalcAcceleratedCmdValues(vector<float>, vector<float> , float);
	// @brief	: 車体速度からホイール速度へ変換する（運動学）
	vector<float> ConvertBodyVel2WheelVels(float, float, float);

	// 指定の範囲外だった場合にその範囲内に収める
	float Clipping(float, float, float);
	// モータをキャリブレーションする
	void CalibrateMotors();



private:
	vector<float> m_accelerated_control_values;
};



#endif /* INC_MOTORCONTROLLER_H_ */
