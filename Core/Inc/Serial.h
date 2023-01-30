/*
 * Serial.h
 *
 *  Created on: 2021/02/16
 *      Author: renew
 */

#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

//#include <string>
#include <vector>
#include <string>
#include "stm32f4xx.h"
#include "stdio.h"  // Encoder
#include "string.h" // Encoder
#include "Define.h"


using namespace std;

class Serial
{
public:
	Serial();
	// @brief	: 受信バッファから1文字取得して内部情報を更新する
	void Update();

	int ReadBuffQtyDMA();

	uint8_t GetBuffDMA();

	void StockString(string p_received_str);
	void StockStringDMA();
	// @brief	: 確定された1行分の文字列を速度指令値に変換する（メンバ変数使用）
	void ConvertReceivedString2SpeedControlValues();
	// @brief	: 速度指令値を取得する
	vector<float> GetSpeedControlValues();

public:
	// 受信できたかの判定に用いる
	HAL_StatusTypeDef m_is_received;
	// 受信した文字を格納する
	int m_read_index_end;
	uint8_t serialData[SERIAL_RECEIVE_BUFFER_SIZE];
	//char m_received_buffer[1];
	// 格納した文字を繋げた文字列にする
	string m_receiving_str;
	// \nで確定された1行分の文字列
	string m_received_str;
	// 文字列から変換された速度指令値
	vector<float> m_speed_control_values;

};



#endif /* INC_SERIAL_H_ */
