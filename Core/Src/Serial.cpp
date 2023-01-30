/*
 * Serial.cpp
 *
 *  Created on: 2021/02/16
 *      Author: renew
 */

#include "CommonMath.h"
#include "Serial.h"
#include "usart.h"
#include "regex"

// DMAの初期化いれる場合，Serial Classをグローバルで宣言するとDMA使えないから注意
Serial::Serial()
{
	// 今のところ3軸限定で使用している
	m_speed_control_values.push_back(0);
	m_speed_control_values.push_back(0);
	m_speed_control_values.push_back(0);

	m_received_str = "0,0,0";
	m_read_index_end = 0;
	for(int i=0; i<SERIAL_RECEIVE_BUFFER_SIZE; i++)
		serialData[i] = 0;
	// DMA
	HAL_UART_Receive_DMA(&huart2, serialData, SERIAL_RECEIVE_BUFFER_SIZE);
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 受信のリングバッファの未読データ数を取得する
// @date	: 2021/03/06
//
// @return			: 未読データ数
//---------------------------------------------------------------------------------------------------------------------
int Serial::ReadBuffQtyDMA(){
	// indexを取得する
	int read_index_begin = huart2.hdmarx->Instance->NDTR; //index取得
	//受信データの先頭位置を計算する
	read_index_begin = SERIAL_RECEIVE_BUFFER_SIZE - read_index_begin;
	//読み込んでいないデータの数を計算する
	int read_buff_qty = read_index_begin - m_read_index_end;
	if(read_buff_qty < 0)
	{
		read_buff_qty += SERIAL_RECEIVE_BUFFER_SIZE;
	}
	return read_buff_qty;
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 受信バッファから1文字取得する
// @date	: 2021/03/06
//
// @return			: 取得した受信データ
//---------------------------------------------------------------------------------------------------------------------
uint8_t Serial::GetBuffDMA(){//データ受信
	uint8_t read_char = 0;

	// indexを取得する
	int read_index_begin = huart2.hdmarx->Instance->NDTR;
	read_index_begin = SERIAL_RECEIVE_BUFFER_SIZE - read_index_begin;

	// 読み込んでいないデータ数を計算する
	int read_buff_qty = read_index_begin - m_read_index_end;
	if(read_buff_qty < 0)
	{
		read_buff_qty += SERIAL_RECEIVE_BUFFER_SIZE;
	}

	// 読み込んでいないデータ数が無い場合はおわり
	if(read_buff_qty < 1)
	{
		return read_char;
	}
	// バッファから1文字取得する
	read_char = serialData[m_read_index_end];
	// リングバッファのインデックスを1つ上げる
	m_read_index_end++;
	// 容量サイズを超えたら0に戻す
	if(m_read_index_end == SERIAL_RECEIVE_BUFFER_SIZE)
	{
		m_read_index_end = 0;
	}

	return read_char;
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 受信バッファから取得した1文字を格納する
// @date	: 2021/02/18
//
// @param1[in]	: []
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void Serial::StockString(string p_received_str)
{
	m_receiving_str += p_received_str;
}

void Serial::StockStringDMA()
{
	int buff_qty =ReadBuffQtyDMA();
	string serial_str;
	for(int i=0; i < buff_qty; i++)
	{
		serial_str += GetBuffDMA();
	}
	m_receiving_str += serial_str;
}


//---------------------------------------------------------------------------------------------------------------------
// @brief	: 受信バッファから1文字取得して内部情報を更新する
// @date	: 2021/02/18
//
// @param1[in]	: []
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void Serial::Update()
{
	char msg[256];
	// DMAでシリアル受信する
	StockStringDMA();

    //sprintf(msg, "ing size %d -> [%s]\n", m_receiving_str.size(), m_receiving_str.c_str());
    //HAL_UART_Transmit( &huart2, (uint8_t *)msg, strlen(msg), 0xFFFF);

	if(m_receiving_str.size() < 6)
	{
		return;
	}

    // 文字列の最後が終端文字のとき
    bool is_serial_line_end = false;
    if(m_receiving_str[m_receiving_str.size()-1] == '\n' || m_receiving_str[m_receiving_str.size()-1] == '\r')
    {
    	is_serial_line_end = true;
    }
	// 文字列を終端文字で分割する
	vector<string> splited_strings =  SplitString2Token(m_receiving_str, '\r', '\n');

	// 表示用
	//sprintf(msg, "------%d------\n", splited_strings.size());
	//HAL_UART_Transmit( &huart2, (uint8_t *)msg, strlen(msg) + 1, 0xFFFF);
	//for(int i=0;i<splited_strings.size(); i++)
	//{
	//	sprintf(msg, "split %d -> [%s]", i, splited_strings[i].c_str());
	//	HAL_UART_Transmit( &huart2, (uint8_t *)msg, strlen(msg) + 1, 0xFFFF);
	//}


	// 途中の文字列を残しておく
	if(splited_strings.size() < 1)
	{
		return;
	}

	// 最後が終端文字のときは1ライン分受信し終わったとき
	string target_strings;
	if(is_serial_line_end == true)
	{
		m_receiving_str = "";
		target_strings = splited_strings[splited_strings.size() - 1];
	}
	// 終端文字じゃないときは受信が1ライン分の途中のとき
	else
	{
		// 途中までの文字列を残しておく
		m_receiving_str = splited_strings[splited_strings.size()-1];

		// 中途半端なのしかなかったとき
		if(splited_strings.size() == 1)
		{
			return;
		}
		else
		{
			// 途中の文字列の1つ前
			target_strings = splited_strings[splited_strings.size() - 2];
		}
	}

	// 各軸の文字列
	vector<string> axis_value_strings = {"", "", ""};
	// 解析している軸の番号
	int axis_idx = 0;



	// 【ここはデバッグ用じゃないから，コメントアウトしないように注意】
	// 文字列⇒数値に変換して格納する
	sscanf(target_strings.c_str(), "%f,%f,%f", &m_speed_control_values[0], &m_speed_control_values[1], &m_speed_control_values[2]);


	//sprintf(msg, "target str [%s]\n", target_strings.c_str());
	//HAL_UART_Transmit( &huart2, (uint8_t *)msg, strlen(msg), 0xFFFF);
    // 配列サイズより大きい場合はエラーになるから注意
    //sprintf(msg, "[  %5.5f %5.5f %5.5f  ] \r\n", m_speed_control_values[0], m_speed_control_values[1], m_speed_control_values[2]);
    //HAL_UART_Transmit( &huart2, (uint8_t *)msg, strlen(msg), 0xFFFF);
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 確定された1行分の文字列を速度指令値に変換する（メンバ変数使用）
// @date	: 2021/02/11
//
// @param1[in]	: 無し
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void Serial::ConvertReceivedString2SpeedControlValues()
{
	sscanf(m_received_str.c_str(), "%f,%f,%f", &m_speed_control_values[0], &m_speed_control_values[1], &m_speed_control_values[2]);
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 速度指令値を取得する
// @date	: 2021/02/11
//
// @param1[in]	: []
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
vector<float> Serial::GetSpeedControlValues()
{
	// 確定された1行分の文字列を速度指令値に変換する
	//ConvertReceivedString2SpeedControlValues();
	return m_speed_control_values;
}
