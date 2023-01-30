/*
 * CommonMath.cpp
 *
 *  Created on: Feb 15, 2021
 *      Author: reo nagai
 */

#include <CommonMath.h>
#include "Define.h"

#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <math.h>


//---------------------------------------------------------------------------------------------------------------------
// @brief	: 指定の範囲外だった場合にその範囲内に収める
// @date	: 2021/02/11
//
// @param1[in]	: クリップする値
// @param1[in]	: 範囲の下限
// @param1[in]	: 範囲の上限
// @return			: 指定の範囲内にクリップされた値
//---------------------------------------------------------------------------------------------------------------------
float Clipping(float val, float lower_bound, float upper_bound)
{
    if(val < lower_bound) {
        val = lower_bound;
    } else if(upper_bound < val) {
        val = upper_bound;
    }
    return val;
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 指定の範囲内にあるかを確認する
// @date	: 2021/02/11
//
// @param1[in]	: 判定する値
// @param1[in]	: 範囲の下限
// @param1[in]	: 範囲の上限
// @return			: 指定の範囲内にあるかの判定
//---------------------------------------------------------------------------------------------------------------------
bool IsClip(float val, float lower_bound, float upper_bound)
{
    if(val < lower_bound) {
        return false;
    } else if(upper_bound < val) {
        return false;
    }
    return true;
}


//---------------------------------------------------------------------------------------------------------------------
// @brief		:  -pi~piの範囲に収める
// @date		: 2021/02/11
//
// @param1[in]	: 角度[deg]
// @return		: -pi~piに収められた角度[rad]
//---------------------------------------------------------------------------------------------------------------------
float RingEnclose(float p_rad)
{
    while(p_rad <= (-M_PI / 2.0)) {
        p_rad += M_PI;
    }
    while((M_PI / 2.0) < p_rad) {
        p_rad -= M_PI;
    }
    return p_rad;
}


//---------------------------------------------------------------------------------------------------------------------
// @brief	: 0~360を-180~180に変換する
// @date	: 2021/02/11
//
// @param1[in]	: 角度[deg]
// @return			: 変換された角度[deg]
//---------------------------------------------------------------------------------------------------------------------
float ChangeRange360to180(float p_rad)
{
    p_rad = RingEnclose(p_rad);
    p_rad -= M_PI / 2.0;

    return p_rad;
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: DegreeからRadianに変換する
// @date	: 2021/02/11
//
// @param1[in]	: 角度[deg]
// @return			: 角度[rad]
//---------------------------------------------------------------------------------------------------------------------
float Deg2Rad(float p_deg)
{
    float k = 1.0 / 180.0;
    return p_deg * k * M_PI;
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: RadianからDegreeに変換する
// @date	: 2021/02/11
//
// @param1[in]	: 角度[rad]
// @return			: 角度[deg]
//---------------------------------------------------------------------------------------------------------------------
float Rad2Deg(float p_rad)
{
    float k = 1.0 / M_PI;
    return p_rad * k * 180;
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 3次元のノルムを計算する
// @date	: 2021/02/11
//
// @param1[in]	: ベクトルx
// @param2[in]	: ベクトルy
// @param3[in]	: ベクトルz
// @return			: 3次元のノルム
//---------------------------------------------------------------------------------------------------------------------
float CalcXYZNorm(float x, float y, float z)
{
    return sqrt(x*x + y*y + z*z);
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 3次元の単位ベクトルを取得する
// @date	: 2021/02/11
//
// @param1[in]	: ベクトルx
// @param2[in]	: ベクトルy
// @param3[in]	: ベクトルz
// @return			: 3次元の単位ベクトル(x,y,z)
//---------------------------------------------------------------------------------------------------------------------
vector<float> CalcUnitVector(float x, float y, float z)
{
    float norm_xyz = CalcXYZNorm(x,y,z);
    vector<float> unit_vector;

    float unit_x = x / norm_xyz;
    float unit_y = y / norm_xyz;
    float unit_z = z / norm_xyz;

    unit_vector.push_back(unit_x);
    unit_vector.push_back(unit_y);
    unit_vector.push_back(unit_z);

    return unit_vector;
}


//---------------------------------------------------------------------------------------------------------------------
// @brief	: 車輪回転速度[rpm]から車輪速度[m/s]へ変換する(GM6020用)
// @date	: 2021/04/29
//
// @param1[in]	: 回転速度[rpm]
// @param1[in]	: 車輪直径[m]
// @return			: 車輪速度[m/s]
//---------------------------------------------------------------------------------------------------------------------
float ConvertWheelRPM2Velocity(int p_wheel_rpm, float p_wheel_diameter)
{
	// 外周[m]
	float periphery = p_wheel_diameter * M_PI;
	// [rps]
	float rps = (float)p_wheel_rpm / 60.0;
	// [m/s]
	return rps * periphery;
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
void ConvertBodyVel2WheelVel(float p_vx, float p_vy, float p_omega, float* p_vel_front, float* p_vel_left_back, float* p_vel_right_back)
{
    // 中心からホイールまでの距離 113.95 [mm]
    float L = 0.1462; // [m] // 124.0

    // 車輪Frontがy軸に正
    //*p_vel_front			= -1.0*p_vx -            0.0 *p_vy - L*p_omega;
    //*p_vel_left_back 	=  0.5*p_vx -  0.8660254*p_vy - L*p_omega;
    //*p_vel_right_back =  0.5*p_vx + 0.8660254*p_vy - L*p_omega;
	// 車輪Frontがx軸に正
    *p_vel_front      =           0*p_vx +  1.0*p_vy  + L*p_omega;
    *p_vel_left_back  =  -0.8660254*p_vx -  0.5*p_vy  + L*p_omega;
    *p_vel_right_back =   0.8660254*p_vx -  0.5*p_vy  + L*p_omega;
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: ホイール速度から車体速度へ変換する（逆運動学）
// @date	: 2021/02/11
//
// @param1[in]	: 車輪速度前
// @param2[in]	: 車輪速度左後
// @param3[in]	: 車輪速度右後
// @param4[out]: 車体速度x
// @param5[out]: 車体速度y
// @param6[out]: 車体角速度 [rad]
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void ConvertWheelVel2BodyVel(float p_vel_front, float p_vel_left_back, float p_vel_right_back, float* p_vx, float* p_vy, float* p_omega)
{
    // 逆行列
	// 中心からホイールまでの距離を変える場合は3項目の係数だけ変えればよい

	// 車輪Frontがy軸に正
    //*p_vx		= -0.666666667 * p_vel_front + 0.333333333 * p_vel_left_back  + 0.333333333 * p_vel_right_back;
    //*p_vy		=          0.0 * p_vel_front - 0.577350272 * p_vel_left_back  + 0.577350272 * p_vel_right_back;
    //*p_omega	=  2.279981760 * p_vel_front + 2.279981760 * p_vel_left_back  +  2.279981760 * p_vel_right_back;


	// 車輪Frontがx軸に正
	//float L_inv = 2.2799817602;
	float L_inv = 2.18710655;
	//float L_inv = 2.331;
    *p_vx		=         0.0 * p_vel_front  - 0.57735271713 * p_vel_left_back + 0.57735271713  * p_vel_right_back;
	*p_vy		= 0.6666666667 * p_vel_front - 0.33333333333 * p_vel_left_back - 0.33333333333  * p_vel_right_back;
	*p_omega	=        L_inv * p_vel_front +        L_inv  * p_vel_left_back +        L_inv   * p_vel_right_back;
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 回転行列をかける
// @date	: 2021/02/11
//
// @param1[in]	: 元座標x
// @param2[in]	: 元座標y
// @param3[in]	: 回転角度[rad]
// @param4[out]: 変換後のx
// @param5[out]: 変換後のy
// @return			: なし
//---------------------------------------------------------------------------------------------------------------------
void Rotation(float p_origin_x, float p_origin_y, float p_body_angle, float* p_result_x, float* p_result_y)
{
    *p_result_x = p_origin_x * cos(p_body_angle) - p_origin_y * sin(p_body_angle);
    *p_result_y = p_origin_x * sin(p_body_angle) + p_origin_y * cos(p_body_angle);
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 文字列を指定の文字で分割する
// @date	: 2021/02/18
//
// @param1[in]	: 分割する文字列
// @param2[in]	: 指定の分割文字
// @return			: 分割された文字列の配列
//---------------------------------------------------------------------------------------------------------------------
vector<string> SplitString(string p_str, char p_token)
{
	// 分割された文字列
	vector<string> splited_strings;
	// 格納用文字列
	string splitting_string;

	for(int idx=0; idx<p_str.size(); idx++)
	{
		if(p_str[idx] == p_token)
		{
			splited_strings.push_back(splitting_string);
			splitting_string = "";
		}
		splitting_string += p_str[idx];
	}

	return splited_strings;
}

//---------------------------------------------------------------------------------------------------------------------
// @brief	: 文字列を指定の2文字で分割する
// @date	: 2021/02/18
//
// @param1[in]	: 分割する文字列
// @param2[in]	: 指定の分割文字1
// @param3[in]	: 指定の分割文字2
// @return			: 分割された文字列の配列
//---------------------------------------------------------------------------------------------------------------------
vector<string> SplitString2Token(string p_str, char p_token1, char p_token2)
{
	// 分割された文字列
	vector<string> splited_strings;
	// 格納用文字列
	string splitting_string;

	for(int idx=0; idx<p_str.size(); idx++)
	{
		// 指定の分割文字なら
		if(p_str[idx] == p_token1 || p_str[idx] == p_token2)
		{
			if(splitting_string.size() < 1)
			{
				continue;
			}
			splited_strings.push_back(splitting_string);
			splitting_string = "";
		}
		// 指定の分割文字は含めない
		else
		{
			splitting_string += p_str[idx];
		}
	}
	// 最後が分割じゃないとき，中途半端に残ったものも入れる
	if(splitting_string.size() > 0)
	{
		splited_strings.push_back(splitting_string);
	}

	return splited_strings;
}


