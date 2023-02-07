/*
 * CommonMath.h
 *
 *  Created on: Feb 15, 2021
 *      Author: reo nagai
 */

#ifndef INC_COMMONMATH_H_
#define INC_COMMONMATH_H_


#include <vector>
#include <string>
using namespace std;

// @brief	: 指定の範囲外だった場合にその範囲内に収める
float Clipping(float val, float lower_bound, float upper_bound);
// @brief	: 指定の範囲内にあるかを確認する
bool IsClip(float val, float lower_bound, float upper_bound);
// @brief	:  -180~180の範囲に収める
float RingEnclose(float p_rad);
// @brief	: 0~360を-180~180に変換する
float ChangeRange360to180(float p_rad);
// @brief	: DegreeからRadianに変換する
float Deg2Rad(float p_deg);
// @brief	: RadianからDegreeに変換する
float Rad2Deg(float p_rad);
// @brief	: 3軸のノルムを計算する
float CalcXYZNorm(float x, float y, float z);
// @brief	: 3軸の単位ベクトルを取得する
vector<float> CalcUnitVector(float x, float y, float z);
// @brief	: 車輪回転速度[rpm]から車輪速度[m/s]へ変換する(GM6020用)
float ConvertWheelRPM2Velocity(int p_wheel_rpm, float p_wheel_diameter);
// @brief	: 車体出力からホイール出力へ変換（運動学）し，回転と並進の出力のレートを加える
void ConvertBodyVel2WheelVelManuaRate(float p_vx, float p_vy, float p_omega, float* p_vel_front, float* p_vel_left_back, float* p_vel_right_back, float translate_rate, float rotate_rate);
// @brief	: 車体速度からホイール速度へ変換する（運動学）
void ConvertBodyVel2WheelVel(float p_vx, float p_vy, float p_omega, float* p_vel_front, float* p_vel_left_back, float* p_vel_right_back);
// @brief	: ホイール速度から車体速度へ変換する（逆運動学）
void ConvertWheelVel2BodyVel(float p_vel_front, float p_vel_left_back, float p_vel_right_back, float* p_vx, float* p_vy, float* p_omega);
// @brief	: 回転行列にかける
void Rotation(float p_origin_x, float p_origin_y, float p_body_angle, float* p_result_x, float* p_result_y);
// @brief	: 文字列を指定の文字で分割する
vector<string> SplitString(string p_str, char p_token);
// @brief	: 文字列を指定の2文字で分割する
vector<string> SplitString2Token(string p_str, char p_token1, char p_token2);


#endif /* INC_COMMONMATH_H_ */
