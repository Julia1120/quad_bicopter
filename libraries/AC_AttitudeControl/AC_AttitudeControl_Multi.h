#pragma once

/// @file    AC_AttitudeControl_Multi.h
/// @brief   ArduCopter attitude control library

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsMulticopter.h>

// default rate controller PID gains
#ifndef AC_ATC_MULTI_RATE_RP_P
  # define AC_ATC_MULTI_RATE_RP_P           0.135f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_I
  # define AC_ATC_MULTI_RATE_RP_I           0.135f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_D
  # define AC_ATC_MULTI_RATE_RP_D           0.0036f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_IMAX
 # define AC_ATC_MULTI_RATE_RP_IMAX         0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_RPY_FILT_HZ
 # define AC_ATC_MULTI_RATE_RPY_FILT_HZ      20.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_P
 # define AC_ATC_MULTI_RATE_YAW_P           0.180f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_I
 # define AC_ATC_MULTI_RATE_YAW_I           0.018f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_D
 # define AC_ATC_MULTI_RATE_YAW_D           0.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_IMAX
 # define AC_ATC_MULTI_RATE_YAW_IMAX        0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_FILT_HZ
 # define AC_ATC_MULTI_RATE_YAW_FILT_HZ     2.5f
#endif


class AC_AttitudeControl_Multi : public AC_AttitudeControl {
public:
	AC_AttitudeControl_Multi(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsMulticopter& motors);

	// empty destructor to suppress compiler warning
	virtual ~AC_AttitudeControl_Multi() {}

    // pid accessors
    AC_PID& get_rate_roll_pid() override { return _pid_rate_roll; }
    AC_PID& get_rate_pitch_pid() override { return _pid_rate_pitch; }
    AC_PID& get_rate_yaw_pid() override { return _pid_rate_yaw; }
    const AC_PID& get_rate_roll_pid() const override { return _pid_rate_roll; }
    const AC_PID& get_rate_pitch_pid() const override { return _pid_rate_pitch; }
    const AC_PID& get_rate_yaw_pid() const override { return _pid_rate_yaw; }

    // Update Alt_Hold angle maximum
    void update_althold_lean_angle_max(float throttle_in) override;

    // Set output throttle
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

    // calculate total body frame throttle required to produce the given earth frame throttle
    float get_throttle_boosted(float throttle_in);

    float cal_dynamic_arm_pitch_intertia(float arm_angle_deg);//计算移动机臂俯仰转动惯量
    float cal_dynamic_arm_roll_intertia(float arm_angle_deg);//计算移动机臂滚转转动惯量

    float cal_dynamic_pitch_intertia(float arm_angle_deg);//计算整机俯仰转动惯量
    float cal_dynamic_roll_intertia(float arm_angle_deg);//计算整机滚转转动惯量
    float cal_dynamic_pitch_trust_coefficient();//计算构型俯仰信任系数
    float cal_dynamic_roll_trust_coefficient();//计算构型滚转信任系数

    void dynamic_pid();//动态调整pid的值
    float get_arm_angle_degrees();//将舵机角度换算成机臂角度，返回的是两机臂夹角

    /*
    //实飞版数据
    float arm_angle_degrees;//两机臂夹角
    
    float f_rcarm_in;//6通道信号值的浮点数形式
    float pwm_min=975;//舵机接受的pwm信号最小值
    float pwm_max=1875;//舵机接受的pwm信号最大值
    float upservo_mid_value=1500;//前倾转舵机中位值
    float servo_pwm_value;//舵机接收的信号值
    float servo_angle_degrees;//舵机与固定机臂夹角
    float arm_angle_degrees_1;//机臂夹角

    //float upservo_value;//矢量电机倾转角
    
    float m_motor_kg=0.01956;//电机质量，kg
    float J_motor_roll=1.128738;//电机转动惯量
    float J_motor_pitch=1.533221;//电机转动惯量

    float m_movearm_kg=0.025;//移动机臂质量，kg
    float d_arm_to_G=29.08;  //移动机臂到重心的距离，mm
    float l_arm=120.0;       //移动机臂臂长
    float long_arm=262.0;    //移动机臂全长
    float w_arm=22;          //移动机臂宽度
    float h_upservo=79.01;   //矢量电机重心距离整机重心的高度
    float J_movearm_xx=1.215;
    float J_movearm_yy=134.769;

    float J_pitch_body=3103.388;//机身部分x轴转动惯量
    float J_roll_body=1340.056;//机身部分y轴转动惯量

    float Kt_pitch;//俯仰信任系数
    float Kt_roll;//滚转信任系数
    float J_pitch_quad=3152.987;//四旋翼整体x轴转动惯量
    float J_pitch_dual=3932.650;//双旋翼整体x轴转动惯量
    float J_roll_quad=2169.380;//四旋翼整体y轴转动惯量
    float J_roll_dual=1390.614;//双旋翼整体y轴转动惯量
    */

    //测力天平版数据
    float arm_angle_degrees;//两机臂夹角
    
    float f_rcarm_in;//6通道信号值的浮点数形式
    float pwm_min=950;//舵机接受的pwm信号最小值
    float pwm_max=1925;//舵机接受的pwm信号最大值
    float upservo_mid_value=1600;//前倾转舵机中位值
    float servo_pwm_value;//舵机接收的信号值
    float servo_angle_degrees;//舵机与固定机臂夹角
    float arm_angle_degrees_1;//机臂夹角

    //float upservo_value;//矢量电机倾转角
    
    float m_motor_kg=0.01956;//电机质量，kg
    float J_motor_roll=1.128738;//电机转动惯量
    float J_motor_pitch=1.533221;//电机转动惯量

    float m_movearm_kg=0.036;//移动机臂质量，kg
    float d_arm_to_G=17.47;  //移动机臂到重心的距离，mm
    float l_arm=158.0;       //移动机臂臂长
    float long_arm=341.0;    //移动机臂全长
    float w_arm=25.0;          //移动机臂宽度
    float h_upservo=79.47;   //矢量电机重心距离整机重心的高度
    float J_movearm_xx=23.832;
    float J_movearm_yy=1417.816;

    float J_pitch_body=4409.285;//机身部分x轴转动惯量
    float J_roll_body=1305.443;//机身部分y轴转动惯量

    float Kt_pitch;//俯仰信任系数
    float Kt_roll;//滚转信任系数
    float J_pitch_quad=4448.133;//四旋翼整体x轴转动惯量
    float J_pitch_dual=5841.940;//双旋翼整体x轴转动惯量
    float J_roll_quad=2738.099;//四旋翼整体y轴转动惯量
    float J_roll_dual=1344.114;//双旋翼整体y轴转动惯量
    //测力天平版数据结束

    //test
    //测力天平版数据
    
    //四旋翼模式初始值
    float init_quad_roll_kp=0.1;
    float init_quad_roll_ki=0.01;
    float init_quad_roll_kd=0.001;
    
    float init_quad_pitch_kp=0.08;
    float init_quad_pitch_ki=0.01;
    float init_quad_pitch_kd=0.001;

    //双旋翼模式初始值
    float init_dual_roll_kp=0.15;
    float init_dual_roll_ki=0.12;
    float init_dual_roll_kd=0.008;
    
    float init_dual_pitch_kp=0.15;
    float init_dual_pitch_ki=0.013;
    float init_dual_pitch_kd=0.008;

    //过度模式参数
    //quad roll
    float quad_roll_kp=0.08;
    float quad_roll_ki=0.01;
    float quad_roll_kd=0.001;
    //quad pitch
    float quad_pitch_kp=0.06;
    float quad_pitch_ki=0.01;
    float quad_pitch_kd=0.001;

    //dual roll
    float dual_roll_kp=0.12;
    float dual_roll_ki=0.15;
    float dual_roll_kd=0.006;
    //dual pitch
    float dual_pitch_kp=0.12;
    float dual_pitch_ki=0.15;
    float dual_pitch_kd=0.006;
    //test end

    /*
    //测力天平版数据
    //四旋翼模式初始值
    float init_quad_roll_kp=0.1;
    float init_quad_roll_ki=0.01;
    float init_quad_roll_kd=0.001;
    
    float init_quad_pitch_kp=0.08;
    float init_quad_pitch_ki=0.01;
    float init_quad_pitch_kd=0.001;

    //双旋翼模式初始值
    float init_dual_roll_kp=0.15;
    float init_dual_roll_ki=0.02;
    float init_dual_roll_kd=0.002;
    
    float init_dual_pitch_kp=0.12;
    float init_dual_pitch_ki=0.012;
    float init_dual_pitch_kd=0.001;

    //过度模式参数
    //quad roll
    float quad_roll_kp=0.12;
    float quad_roll_ki=0.02;
    float quad_roll_kd=0.002;
    //quad pitch
    float quad_pitch_kp=0.1;
    float quad_pitch_ki=0.01;
    float quad_pitch_kd=0.001;

    //dual roll
    float dual_roll_kp=0.18;
    float dual_roll_ki=0.05;
    float dual_roll_kd=0.005;
    //dual pitch
    float dual_pitch_kp=0.15;
    float dual_pitch_ki=0.02;
    float dual_pitch_kd=0.005;
    //测力天平版数据结束
    */

    float roll_kp;
    float roll_ki;
    float roll_kd;

    float pitch_kp;
    float pitch_ki;
    float pitch_kd;

    float yaw_kp;
    float yaw_ki;
    float yaw_kd;

    int16_t             rcarmin_output();//根据8通道pwm值决定转动机臂命令是由旋钮发出还是拨杆发出
    uint16_t            rcarm_in_read;//旋钮控制机臂旋转通道pwm值6通道
    // uint16_t            rcarm_in_calculate;//拨杆切换时计算得到控制机臂旋转通道pwm值9通道
    uint16_t            rcarm_in_output;//用于下一步计算的控制机臂旋转通道pwm值

    // set desired throttle vs attitude mixing (actual mix is slewed towards this value over 1~2 seconds)
    //  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
    //  has no effect when throttle is above hover throttle
    void set_throttle_mix_min() override { _throttle_rpy_mix_desired = _thr_mix_min; }
    void set_throttle_mix_man() override { _throttle_rpy_mix_desired = _thr_mix_man; }
    void set_throttle_mix_max(float ratio) override;
    void set_throttle_mix_value(float value) override { _throttle_rpy_mix_desired = _throttle_rpy_mix = value; }
    float get_throttle_mix(void) const override { return _throttle_rpy_mix; }

    // are we producing min throttle?
    bool is_throttle_mix_min() const override { return (_throttle_rpy_mix < 1.25f * _thr_mix_min); }

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run() override;

    // sanity check parameters.  should be called once before take-off
    void parameter_sanity_check() override;

    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // boost angle_p/pd each cycle on high throttle slew
    void update_throttle_gain_boost();

    // update_throttle_rpy_mix - updates thr_low_comp value towards the target
    void update_throttle_rpy_mix();

    // get maximum value throttle can be raised to based on throttle vs attitude prioritisation
    float get_throttle_avg_max(float throttle_in);

    AP_MotorsMulticopter& _motors_multi;
    AC_PID                _pid_rate_roll {
        AC_PID::Defaults{
            .p         = AC_ATC_MULTI_RATE_RP_P,
            .i         = AC_ATC_MULTI_RATE_RP_I,
            .d         = AC_ATC_MULTI_RATE_RP_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_MULTI_RATE_RP_IMAX,
            .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .filt_E_hz = 0.0f,
            .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };
    AC_PID                _pid_rate_pitch{
        AC_PID::Defaults{
            .p         = AC_ATC_MULTI_RATE_RP_P,
            .i         = AC_ATC_MULTI_RATE_RP_I,
            .d         = AC_ATC_MULTI_RATE_RP_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_MULTI_RATE_RP_IMAX,
            .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .filt_E_hz = 0.0f,
            .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };

    AC_PID                _pid_rate_yaw{
        AC_PID::Defaults{
            .p         = AC_ATC_MULTI_RATE_YAW_P,
            .i         = AC_ATC_MULTI_RATE_YAW_I,
            .d         = AC_ATC_MULTI_RATE_YAW_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_MULTI_RATE_YAW_IMAX,
            .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .filt_E_hz = AC_ATC_MULTI_RATE_YAW_FILT_HZ,
            .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };

    AP_Float              _thr_mix_man;     // throttle vs attitude control prioritisation used when using manual throttle (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_min;     // throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_max;     // throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)

    // angle_p/pd boost multiplier
    AP_Float              _throttle_gain_boost;
};
