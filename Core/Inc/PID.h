/*
 * PID.h
 *
 *  Created on: 2021/02/11
 *      Author: renew
 */

#ifndef INC_PID_H_
#define INC_PID_H_

// 測打型PID
class SpeedTypePIDController
{
    public:
		// 1周期前の現在値と目標値の差分
        float pre_diff_vel;
        // 差分の積分値
        float vel_diff_integral;
        // 制御量の加算値
        float u_sum;
        // 目標値
        float target_velocity;
        // 比例ゲイン
        float Kp;
        // 積分ゲイン
        float Ki;
        // 微分ゲイン
        float Kd;

        SpeedTypePIDController();

        // ゲインを設定する
        void SetGain(float kp, float ki, float kd);
        // 目標値を設定する
        void SetTarget(float p_target_velocity);
        // 制御量の計算値を返す
        float GetControlValue();
        // 状態を更新してPIDの制御量を計算する
        void Update(float p_now_velocity);
};

// 位置型PID
class PositionTypePIDController
{
    public:
        float pre_diff;
        float diff_integral;
        float u;
        float target;

        float Kp;
        float Ki;
        float Kd;

        PositionTypePIDController();
        // ゲインを設定する
        void SetGain(float kp, float ki, float kd);
        // 目標値を設定する
        void SetTarget(float p_target);
        // 制御量の計算値を返す
        float GetControlValue();
        // 状態を更新してPIDの制御量を計算する
        void Update(float p_now);
};


#endif /* INC_PID_H_ */
