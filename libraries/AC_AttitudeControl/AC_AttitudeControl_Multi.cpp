#include "AC_AttitudeControl_Multi.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>
#include <AP_Scheduler/AP_Scheduler.h>

#include <GCS_MAVLink/GCS.h>
#include <AP_Motors/AP_MotorsMatrix.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Multi::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Corrects in proportion to the difference between the desired roll rate vs actual roll rate
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_RLL_PDMX
    // @DisplayName: Roll axis rate controller PD sum maximum
    // @Description: Roll axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: RAT_RLL_D_FF
    // @DisplayName: Roll Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_RLL_NTF
    // @DisplayName: Roll Target notch filter index
    // @Description: Roll Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: RAT_RLL_NEF
    // @DisplayName: Roll Error notch filter index
    // @Description: Roll Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 1, AC_AttitudeControl_Multi, AC_PID),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Corrects in proportion to the difference between the desired pitch rate vs actual pitch rate output
    // @Range: 0.01 0.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FLTT
    // @DisplayName: Pitch axis rate controller target frequency in Hz
    // @Description: Pitch axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTE
    // @DisplayName: Pitch axis rate controller error frequency in Hz
    // @Description: Pitch axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTD
    // @DisplayName: Pitch axis rate controller derivative frequency in Hz
    // @Description: Pitch axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_SMAX
    // @DisplayName: Pitch slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_PIT_PDMX
    // @DisplayName: Pitch axis rate controller PD sum maximum
    // @Description: Pitch axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: RAT_PIT_D_FF
    // @DisplayName: Pitch Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_PIT_NTF
    // @DisplayName: Pitch Target notch filter index
    // @Description: Pitch Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: RAT_PIT_NEF
    // @DisplayName: Pitch Error notch filter index
    // @Description: Pitch Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 2, AC_AttitudeControl_Multi, AC_PID),

    // @Param: RAT_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Corrects in proportion to the difference between the desired yaw rate vs actual yaw rate
    // @Range: 0.10 2.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.010 1.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FF
    // @DisplayName: Yaw axis rate controller feed forward
    // @Description: Yaw axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FLTT
    // @DisplayName: Yaw axis rate controller target frequency in Hz
    // @Description: Yaw axis rate controller target frequency in Hz
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTE
    // @DisplayName: Yaw axis rate controller error frequency in Hz
    // @Description: Yaw axis rate controller error frequency in Hz
    // @Range: 0 20
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTD
    // @DisplayName: Yaw axis rate controller derivative frequency in Hz
    // @Description: Yaw axis rate controller derivative frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_SMAX
    // @DisplayName: Yaw slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_YAW_PDMX
    // @DisplayName: Yaw axis rate controller PD sum maximum
    // @Description: Yaw axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: RAT_YAW_D_FF
    // @DisplayName: Yaw Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_YAW_NTF
    // @DisplayName: Yaw Target notch filter index
    // @Description: Yaw Target notch filter index
    // @Range: 1 8
    // @Units: Hz
    // @User: Advanced

    // @Param: RAT_YAW_NEF
    // @DisplayName: Yaw Error notch filter index
    // @Description: Yaw Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 3, AC_AttitudeControl_Multi, AC_PID),

    // @Param: THR_MIX_MIN
    // @DisplayName: Throttle Mix Minimum
    // @Description: Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.25
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MIN", 4, AC_AttitudeControl_Multi, _thr_mix_min, AC_ATTITUDE_CONTROL_MIN_DEFAULT),

    // @Param: THR_MIX_MAX
    // @DisplayName: Throttle Mix Maximum
    // @Description: Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.5 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAX", 5, AC_AttitudeControl_Multi, _thr_mix_max, AC_ATTITUDE_CONTROL_MAX_DEFAULT),

    // @Param: THR_MIX_MAN
    // @DisplayName: Throttle Mix Manual
    // @Description: Throttle vs attitude control prioritisation used during manual flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAN", 6, AC_AttitudeControl_Multi, _thr_mix_man, AC_ATTITUDE_CONTROL_MAN_DEFAULT),

    // @Param: THR_G_BOOST
    // @DisplayName: Throttle-gain boost
    // @Description: Throttle-gain boost ratio. A value of 0 means no boosting is applied, a value of 1 means full boosting is applied. Describes the ratio increase that is applied to angle P and PD on pitch and roll.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("THR_G_BOOST", 7, AC_AttitudeControl_Multi, _throttle_gain_boost, 0.0f),

    AP_GROUPEND
};

AC_AttitudeControl_Multi::AC_AttitudeControl_Multi(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsMulticopter& motors) :
    AC_AttitudeControl(ahrs, aparm, motors),
    _motors_multi(motors)
{
    AP_Param::setup_object_defaults(this, var_info);

#if AP_FILTER_ENABLED
    set_notch_sample_rate(AP::scheduler().get_loop_rate_hz());
#endif
}

// Update Alt_Hold angle maximum
void AC_AttitudeControl_Multi::update_althold_lean_angle_max(float throttle_in)
{
    // calc maximum tilt angle based on throttle
    float thr_max = _motors_multi.get_throttle_thrust_max();

    // divide by zero check
    if (is_zero(thr_max)) {
        _althold_lean_angle_max = 0.0f;
        return;
    }

    float althold_lean_angle_max = acosf(constrain_float(throttle_in / (AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX * thr_max), 0.0f, 1.0f));
    _althold_lean_angle_max = _althold_lean_angle_max + (_dt / (_dt + _angle_limit_tc)) * (althold_lean_angle_max - _althold_lean_angle_max);
}

void AC_AttitudeControl_Multi::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in = throttle_in;
    update_althold_lean_angle_max(throttle_in);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (apply_angle_boost) {
        // Apply angle boost
        throttle_in = get_throttle_boosted(throttle_in);
    } else {
        // Clear angle_boost for logging purposes
        _angle_boost = 0.0f;
    }
    _motors.set_throttle(throttle_in);
    _motors.set_throttle_avg_max(get_throttle_avg_max(MAX(throttle_in, _throttle_in)));
}

void AC_AttitudeControl_Multi::set_throttle_mix_max(float ratio)
{
    ratio = constrain_float(ratio, 0.0f, 1.0f);
    _throttle_rpy_mix_desired = (1.0f - ratio) * _thr_mix_min + ratio * _thr_mix_max;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi::get_throttle_boosted(float throttle_in)
{
    if (!_angle_boost_enabled) {
        _angle_boost = 0;
        return throttle_in;
    }
    // inverted_factor is 1 for tilt angles below 60 degrees
    // inverted_factor reduces from 1 to 0 for tilt angles between 60 and 90 degrees

    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
    float inverted_factor = constrain_float(10.0f * cos_tilt, 0.0f, 1.0f);
    float cos_tilt_target = cosf(_thrust_angle);
    float boost_factor = 1.0f / constrain_float(cos_tilt_target, 0.1f, 1.0f);

    float throttle_out = throttle_in * inverted_factor * boost_factor;
    _angle_boost = constrain_float(throttle_out - throttle_in, -1.0f, 1.0f);
    return throttle_out;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi::get_throttle_avg_max(float throttle_in)
{
    throttle_in = constrain_float(throttle_in, 0.0f, 1.0f);
    return MAX(throttle_in, throttle_in * MAX(0.0f, 1.0f - _throttle_rpy_mix) + _motors.get_throttle_hover() * _throttle_rpy_mix);
}

// update_throttle_gain_boost - boost angle_p/pd each cycle on high throttle slew
void AC_AttitudeControl_Multi::update_throttle_gain_boost()
{
    // Boost PD and Angle P on very rapid throttle changes
    if (_motors.get_throttle_slew_rate() > AC_ATTITUDE_CONTROL_THR_G_BOOST_THRESH) {
        const float pd_boost = constrain_float(_throttle_gain_boost + 1.0f, 1.0, 2.0);
        set_PD_scale_mult(Vector3f(pd_boost, pd_boost, 1.0f));

        const float angle_p_boost = constrain_float((_throttle_gain_boost + 1.0f) * (_throttle_gain_boost + 1.0f), 1.0, 4.0);
        set_angle_P_scale_mult(Vector3f(angle_p_boost, angle_p_boost, 1.0f));
    }
}

// update_throttle_rpy_mix - slew set_throttle_rpy_mix to requested value
void AC_AttitudeControl_Multi::update_throttle_rpy_mix()
{
    // slew _throttle_rpy_mix to _throttle_rpy_mix_desired
    if (_throttle_rpy_mix < _throttle_rpy_mix_desired) {
        // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
        _throttle_rpy_mix += MIN(2.0f * _dt, _throttle_rpy_mix_desired - _throttle_rpy_mix);
    } else if (_throttle_rpy_mix > _throttle_rpy_mix_desired) {
        // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
        _throttle_rpy_mix -= MIN(0.5f * _dt, _throttle_rpy_mix - _throttle_rpy_mix_desired);

        // if the mix is still higher than that being used, reset immediately
        const float throttle_hover = _motors.get_throttle_hover();
        const float throttle_in = _motors.get_throttle();
        const float throttle_out = MAX(_motors.get_throttle_out(), throttle_in);
        float mix_used;
        // since throttle_out >= throttle_in at this point we don't need to check throttle_in < throttle_hover
        if (throttle_out < throttle_hover) {
            mix_used = (throttle_out - throttle_in) / (throttle_hover - throttle_in);
        } else {
            mix_used = throttle_out / throttle_hover;
        }

        _throttle_rpy_mix = MIN(_throttle_rpy_mix, MAX(mix_used, _throttle_rpy_mix_desired));
    }
    _throttle_rpy_mix = constrain_float(_throttle_rpy_mix, 0.1f, AC_ATTITUDE_CONTROL_MAX);
}

int16_t AC_AttitudeControl_Multi::rcarmin_output()//根据8通道pwm值决定转动机臂命令是由旋钮发出还是拨杆发出
{
    uint16_t rc8_in=rc().channel(CH_8)->get_radio_in();//读取8通道开关位置信息
    if((rc8_in<=1100))//旋钮控制机臂旋转
    {
        rcarm_in_read=rc().channel(CH_6)->get_radio_in();//读取6通道开关位置信息
        rcarm_in_output=rcarm_in_read;
    }
    else if((rc8_in>=1900))//拨杆控制机臂旋转
    {
        rcarm_in_read=rc().channel(CH_9)->get_radio_in();//读取6通道开关位置信息
        rcarm_in_output=rcarm_in_read;
    }

    return rcarm_in_output;
}

float AC_AttitudeControl_Multi::get_arm_angle_degrees()//将舵机角度换算成机臂角度，返回的是两机臂夹角
{
    uint16_t rcarm_in=rcarmin_output();//读取移动机臂通道开关位置信息
    
    f_rcarm_in=float(rcarm_in);//将6通道信号值转为浮点数

    float k_RC_to_PWM=(pwm_max-pwm_min)/(1100-1900);
    float b_RC_to_PWM=pwm_max-k_RC_to_PWM*1100;
    
    servo_pwm_value=k_RC_to_PWM*f_rcarm_in+b_RC_to_PWM;//求6通道每个信号值对应的输出到舵机的pwm值

    //舵盘直驱计算机臂夹角
    arm_angle_degrees_1=((servo_pwm_value-pwm_min)/(pwm_max-pwm_min))*90;
    gcs().send_text(MAV_SEVERITY_NOTICE, "机臂夹角%f",arm_angle_degrees_1);
    /*
    //四连杆机构计算机臂夹角
    servo_angle_degrees=((servo_pwm_value-pwm_min)/(pwm_max-pwm_min))*73.68+16.27;//根据舵机的pwm值计算舵机与固定机臂夹角

    if(servo_angle_degrees>=16.27&&servo_angle_degrees<=48.62)
    {
        arm_angle_degrees_1=0.0064*servo_angle_degrees*servo_angle_degrees+0.579*servo_angle_degrees-10.817;
    }
    else if(servo_angle_degrees>48.62&&servo_angle_degrees<=72.19)
    {
        arm_angle_degrees_1=0.0076*servo_angle_degrees*servo_angle_degrees+0.35*servo_angle_degrees-4.9661;
    }
    else if(servo_angle_degrees>72.19&&servo_angle_degrees<=89.95)
    {
        arm_angle_degrees_1=0.0145*servo_angle_degrees*servo_angle_degrees-0.6659*servo_angle_degrees+32.357;
    }

    //arm_angle_degrees_1=90*(rcarm_in-1100)/800;
        
    //arm_angle_degrees_1 = cal_arm_angle_degrees(servo_angle_degrees, lower_arm_angle_degrees, upper_arm_angle_degrees);//计算移动机臂的转角

    //四连杆机构计算机臂夹角结束
    */
        
    return arm_angle_degrees_1;

}

//计算扭转机臂俯仰转动惯量
float AC_AttitudeControl_Multi::cal_dynamic_arm_pitch_intertia(float arm_angle_deg)
{
    float move_angle_deg=90-arm_angle_deg;                 //机臂转过角度，绕z轴旋转角度
    float sin_moveangle=sinf(radians(move_angle_deg));
    float cos_moveangle=cosf(radians(move_angle_deg));

    float J_movearm_pitch=J_movearm_xx*cos_moveangle*cos_moveangle
                         +J_movearm_yy*sin_moveangle*sin_moveangle;
    
    return J_movearm_pitch;
}
//计算扭转机臂滚转转动惯量
float AC_AttitudeControl_Multi::cal_dynamic_arm_roll_intertia(float arm_angle_deg)
{
    float move_angle_deg=90-arm_angle_deg;                  //机臂转过角度，绕z轴旋转角度
    float sin_moveangle=sinf(radians(move_angle_deg));
    float cos_moveangle=cosf(radians(move_angle_deg));
                                        
    float J_movearm_roll=J_movearm_xx*sin_moveangle*sin_moveangle
                        +J_movearm_yy*cos_moveangle*cos_moveangle;
    
    return J_movearm_roll;
}


//计算整机俯仰转动惯量
float AC_AttitudeControl_Multi::cal_dynamic_pitch_intertia(float arm_angle_deg)
{
    float _arm_angle_deg=arm_angle_deg;                     //两机臂夹角
    float mov_motor_Y=l_arm*cosf(radians(_arm_angle_deg));

    float J_movearm_pitch=cal_dynamic_arm_pitch_intertia(_arm_angle_deg);

    float J_pitch_all=J_pitch_body
                     +2*(J_motor_pitch+m_motor_kg*(mov_motor_Y*mov_motor_Y+20*20))//20是电机重心到整体重心的距离
                     +J_movearm_pitch+m_movearm_kg*d_arm_to_G*d_arm_to_G;
    
    //gcs().send_text(MAV_SEVERITY_NOTICE, "俯仰转动惯量%f",J_pitch_all);
    float k_pitch=J_pitch_all/(2*l_arm+2*mov_motor_Y);
    //gcs().send_text(MAV_SEVERITY_NOTICE, "俯仰运动系数%f",k_pitch);

    //return J_pitch_all;
    return k_pitch;
}
//计算整机滚转转动惯量
float AC_AttitudeControl_Multi::cal_dynamic_roll_intertia(float arm_angle_deg)
{
    float _arm_angle_deg=arm_angle_deg;                     //两机臂夹角
    float mov_motor_X=l_arm*sinf(radians(_arm_angle_deg));

    float J_movearm_roll=cal_dynamic_arm_roll_intertia(_arm_angle_deg);

    float J_roll_all=J_roll_body
                    +2*(J_motor_roll+m_motor_kg*(mov_motor_X*mov_motor_X+20*20))//20是电机重心到整体重心的距离
                    +J_movearm_roll+m_movearm_kg*d_arm_to_G*d_arm_to_G;

    //float upservo_value=factor.get_tront_servo_value();
    //gcs().send_text(MAV_SEVERITY_NOTICE, "计算%f",upservo_value);
    float upservo_pwm=500*upservo_value+1500;
    //gcs().send_text(MAV_SEVERITY_NOTICE, "倾转舵机pwm%f",upservo_pwm);
    float upservo_angle_degree=fabs(upservo_pwm-1500)*73.68/900;
    //gcs().send_text(MAV_SEVERITY_NOTICE, "舵机倾转角%f",upservo_angle_degree);
    float sin_upservo=sinf(radians(upservo_angle_degree));

    float k_roll=J_roll_all/(2*mov_motor_X+2*h_upservo*sin_upservo);
     
    //gcs().send_text(MAV_SEVERITY_NOTICE, "滚转转动惯量%f",J_roll_all);

    return k_roll;
}


//计算俯仰构型信任系数
float AC_AttitudeControl_Multi::cal_dynamic_pitch_trust_coefficient()
{
    arm_angle_degrees=get_arm_angle_degrees();//两机臂夹角
    // gcs().send_text(MAV_SEVERITY_NOTICE, "机臂夹角%f",arm_angle_degrees);

    //float J_pitch=cal_dynamic_pitch_intertia(arm_angle_degrees);//x轴转动惯量,kg*mm2
    float k_pitch=cal_dynamic_pitch_intertia(arm_angle_degrees);

    float k_pitch_quad=J_pitch_quad/(2*l_arm);
    float k_pitch_dual=J_pitch_dual/(4*l_arm);

    Kt_pitch=fabs(k_pitch-k_pitch_dual)/(fabs(k_pitch-k_pitch_quad)+fabs(k_pitch-k_pitch_dual));
    
    //Kt_pitch=fabs(J_pitch-J_pitch_dual)/(fabs(J_pitch-J_pitch_quad)+fabs(J_pitch-J_pitch_dual));
    //Kt_pitch=-(J_pitch-J_pitch_dual)/((J_pitch-J_pitch_quad)-(J_pitch-J_pitch_dual));

    return Kt_pitch;
}
//计算滚转构型信任系数
float AC_AttitudeControl_Multi::cal_dynamic_roll_trust_coefficient()
{
    arm_angle_degrees=get_arm_angle_degrees();//两机臂夹角
    //float J_roll=cal_dynamic_roll_intertia(arm_angle_degrees);//y轴转动惯量,kg*mm2
    
    //float upservo_value=factor.get_tront_servo_value();
    //float upservo_pwm=500*upservo_value+upservo_mid_value;
    //float upservo_angle_degree=fabs(upservo_pwm-upservo_mid_value)*73.68/900;
    float sin_upservo=sinf(radians(16.4));  //upservo_angle_degree->10

    float k_roll=cal_dynamic_roll_intertia(arm_angle_degrees);

    float k_roll_quad=J_roll_quad/(2*l_arm);
    float k_roll_dual=J_roll_dual/(2*h_upservo*sin_upservo);
    Kt_roll=fabs(k_roll-k_roll_dual)/(fabs(k_roll-k_roll_quad)+fabs(k_roll-k_roll_dual));
    
    //Kt_roll=fabs(J_roll-J_roll_dual)/(fabs(J_roll-J_roll_quad)+fabs(J_roll-J_roll_dual));
    //Kt_roll=(J_roll-J_roll_dual)/(-(J_roll-J_roll_quad)+(J_roll-J_roll_dual));

    return Kt_roll;
}

void AC_AttitudeControl_Multi::dynamic_pid()//动态调整pid的值
{
    /*
    //quad roll
    float quad_roll_kp=0.085;
    float quad_roll_ki=0.02;
    float quad_roll_kd=0.001;
    //quad pitch
    float quad_pitch_kp=0.06;
    float quad_pitch_ki=0.01;
    float quad_pitch_kd=0.001;

    //dual roll
    float dual_roll_kp=0.12;
    float dual_roll_ki=0.02;
    float dual_roll_kd=0.001;
    //dual pitch
    float dual_pitch_kp=0.08;
    float dual_pitch_ki=0.012;
    float dual_pitch_kd=0.001;
    

    uint16_t rcarm_in=rcarmin_output();//读取移动机臂通道开关位置信息
    if(rcarm_in<=1100)
    {
        //roll
        get_rate_roll_pid().kP(constrain_float(quad_roll_kp,quad_roll_kp,dual_roll_kp));
        get_rate_roll_pid().kI(constrain_float(quad_roll_ki,quad_roll_ki,dual_roll_ki));
        get_rate_roll_pid().kD(constrain_float(dual_roll_kd,quad_roll_kd,dual_roll_kd));
        //pitch
        get_rate_pitch_pid().kP(constrain_float(quad_pitch_kp,quad_pitch_kp,dual_pitch_kp));
        get_rate_pitch_pid().kI(constrain_float(quad_pitch_ki,quad_pitch_ki,dual_pitch_ki));
        get_rate_pitch_pid().kD(constrain_float(quad_pitch_kd,quad_pitch_kd,dual_pitch_kd));
    
    }
    else if(rcarm_in<1900&&rcarm_in>1100)
    {
        Kt_roll=cal_dynamic_roll_trust_coefficient();
        //gcs().send_text(MAV_SEVERITY_NOTICE, "滚转信任系数%f",Kt_roll);
        float dynamic_roll_kp=quad_roll_kp*Kt_roll+dual_roll_kp*(1-Kt_roll);
        float dynamic_roll_ki=quad_roll_ki*Kt_roll+dual_roll_ki*(1-Kt_roll);
        float dynamic_roll_kd=quad_roll_kd*Kt_roll+dual_roll_kd*(1-Kt_roll);
        
        Kt_pitch=cal_dynamic_pitch_trust_coefficient();
        //gcs().send_text(MAV_SEVERITY_NOTICE, "俯仰信任函数%f",Kt_pitch);
        float dynamic_pitch_kp=quad_pitch_kp*Kt_pitch+dual_pitch_kp*(1-Kt_pitch);
        float dynamic_pitch_ki=quad_pitch_ki*Kt_pitch+dual_pitch_ki*(1-Kt_pitch);
        float dynamic_pitch_kd=quad_pitch_kd*Kt_pitch+dual_pitch_kd*(1-Kt_pitch);

        //roll
        get_rate_roll_pid().kP(constrain_float(dynamic_roll_kp,quad_roll_kp,dual_roll_kp));
        get_rate_roll_pid().kI(constrain_float(dynamic_roll_ki,quad_roll_ki,dual_roll_ki));
        get_rate_roll_pid().kD(constrain_float(dynamic_roll_kd,quad_roll_kd,dual_roll_kd));
        //pitch
        get_rate_pitch_pid().kP(constrain_float(dynamic_pitch_kp,quad_pitch_kp,dual_pitch_kp));
        get_rate_pitch_pid().kI(constrain_float(dynamic_pitch_ki,quad_pitch_ki,dual_pitch_ki));
        get_rate_pitch_pid().kD(constrain_float(dynamic_pitch_kd,quad_pitch_kd,dual_pitch_kd));
        
        
        //gcs().send_text(MAV_SEVERITY_NOTICE, "俯仰ki%f",dynamic_pitch_ki);
        
    }
    else if(rcarm_in>=1900)
    {
        //roll
        get_rate_roll_pid().kP(constrain_float(dual_roll_kp,quad_roll_kp,dual_roll_kp));
        get_rate_roll_pid().kI(constrain_float(dual_roll_ki,quad_roll_ki,dual_roll_ki));
        get_rate_roll_pid().kD(constrain_float(dual_roll_kd,quad_roll_kd,dual_roll_kd));
        //pitch
        get_rate_pitch_pid().kP(constrain_float(dual_pitch_kp,quad_pitch_kp,dual_pitch_kp));
        get_rate_pitch_pid().kI(constrain_float(dual_pitch_ki,quad_pitch_ki,dual_pitch_ki));
        get_rate_pitch_pid().kD(constrain_float(dual_pitch_kd,quad_pitch_kd,dual_pitch_kd));
    
    }
    */
   float pitch_kp_min=MIN(init_quad_pitch_kp,quad_pitch_kp);
   float pitch_kp_max=MIN(init_dual_pitch_kp,dual_pitch_kp);

   float pitch_ki_min=MIN(init_quad_pitch_ki,quad_pitch_ki);
   float pitch_ki_max=MIN(init_dual_pitch_ki,dual_pitch_ki);

   float pitch_kd_min=MIN(init_quad_pitch_kd,quad_pitch_kd);
   float pitch_kd_max=MIN(init_dual_pitch_kd,dual_pitch_kd);


   float roll_kp_min=MIN(init_quad_roll_kp,quad_roll_kp);
   float roll_kp_max=MIN(init_dual_roll_kp,dual_roll_kp);

   float roll_ki_min=MIN(init_quad_roll_ki,quad_roll_ki);
   float roll_ki_max=MIN(init_dual_roll_ki,dual_roll_ki);

   float roll_kd_min=MIN(init_quad_roll_kd,quad_roll_kd);
   float roll_kd_max=MIN(init_dual_roll_kd,dual_roll_kd);


    uint16_t rcarm_in=rcarmin_output();//读取移动机臂通道开关位置信息
    if(rcarm_in<=1100)
    {
        roll_kp=init_quad_roll_kp;
        roll_ki=init_quad_roll_ki;
        roll_kd=init_quad_roll_kd;

        pitch_kp=init_quad_pitch_kp;
        pitch_ki=init_quad_pitch_ki;
        pitch_kd=init_quad_pitch_kd;
        //roll
        get_rate_roll_pid().kP(constrain_float(roll_kp, roll_kp_min,roll_kp_max));
        get_rate_roll_pid().kI(constrain_float(roll_ki, roll_ki_min,roll_ki_max));
        get_rate_roll_pid().kD(constrain_float(roll_kd, roll_kd_min,roll_kd_max));
        //pitch
        get_rate_pitch_pid().kP(constrain_float(pitch_kp, pitch_kp_min,pitch_kp_max));
        get_rate_pitch_pid().kI(constrain_float(pitch_ki, pitch_ki_min,pitch_ki_max));
        get_rate_pitch_pid().kD(constrain_float(pitch_kd, pitch_kd_min,pitch_kd_max));
    
    }

    /*
    else if(rcarm_in<1900&&rcarm_in>1100)
    {
        arm_angle_degrees=get_arm_angle_degrees();

        Kt_roll=cal_dynamic_roll_trust_coefficient();
        gcs().send_text(MAV_SEVERITY_NOTICE, "滚转信任系数%f",Kt_roll);

        Kt_pitch=cal_dynamic_pitch_trust_coefficient();
        gcs().send_text(MAV_SEVERITY_NOTICE, "俯仰信任函数%f",Kt_pitch);

        //if(arm_angle_degrees<90&&arm_angle_degrees>=47.5875)
        //{
            float dynamic_roll_kp=init_quad_roll_kp*Kt_roll+init_dual_roll_kp*(1-Kt_roll);
            float dynamic_roll_ki=init_quad_roll_ki*Kt_roll+init_dual_roll_ki*(1-Kt_roll);
            float dynamic_roll_kd=init_quad_roll_kd*Kt_roll+init_dual_roll_kd*(1-Kt_roll);
       
            float dynamic_pitch_kp=init_quad_pitch_kp*Kt_pitch+init_dual_pitch_kp*(1-Kt_pitch);
            float dynamic_pitch_ki=init_quad_pitch_ki*Kt_pitch+init_dual_pitch_ki*(1-Kt_pitch);
            float dynamic_pitch_kd=init_quad_pitch_kd*Kt_pitch+init_dual_pitch_kd*(1-Kt_pitch);

            roll_kp=dynamic_roll_kp;
            roll_ki=dynamic_roll_ki;
            roll_kd=dynamic_roll_kd;

            pitch_kp=dynamic_pitch_kp;
            pitch_ki=dynamic_pitch_ki;
            pitch_kd=dynamic_pitch_kd;
        //}
        
        else if(arm_angle_degrees<47.5875&&arm_angle_degrees>0.0)
        {
        
            //Kt_roll=cal_dynamic_roll_trust_coefficient();
            //gcs().send_text(MAV_SEVERITY_NOTICE, "滚转信任系数%f",Kt_roll);
            float dynamic_roll_kp=quad_roll_kp*Kt_roll+dual_roll_kp*(1-Kt_roll);
            float dynamic_roll_ki=quad_roll_ki*Kt_roll+dual_roll_ki*(1-Kt_roll);
            float dynamic_roll_kd=quad_roll_kd*Kt_roll+dual_roll_kd*(1-Kt_roll);
       
            //Kt_pitch=cal_dynamic_pitch_trust_coefficient();
            //gcs().send_text(MAV_SEVERITY_NOTICE, "俯仰信任函数%f",Kt_pitch);
            float dynamic_pitch_kp=quad_pitch_kp*Kt_pitch+dual_pitch_kp*(1-Kt_pitch);
            float dynamic_pitch_ki=quad_pitch_ki*Kt_pitch+dual_pitch_ki*(1-Kt_pitch);
            float dynamic_pitch_kd=quad_pitch_kd*Kt_pitch+dual_pitch_kd*(1-Kt_pitch);

            roll_kp=dynamic_roll_kp;
            roll_ki=dynamic_roll_ki;
            roll_kd=dynamic_roll_kd;

            pitch_kp=dynamic_pitch_kp;
            pitch_ki=dynamic_pitch_ki;
            pitch_kd=dynamic_pitch_kd;
        }
        

        //roll
        get_rate_roll_pid().kP(constrain_float(roll_kp,init_quad_roll_kp,dual_roll_kp));
        get_rate_roll_pid().kI(constrain_float(roll_ki,init_quad_roll_ki,dual_roll_ki));
        get_rate_roll_pid().kD(constrain_float(roll_kd,init_quad_roll_kd,dual_roll_kd));
        //pitch
        get_rate_pitch_pid().kP(constrain_float(pitch_kp,init_quad_pitch_kp,dual_pitch_kp));
        get_rate_pitch_pid().kI(constrain_float(pitch_ki,init_quad_pitch_ki,dual_pitch_ki));
        get_rate_pitch_pid().kD(constrain_float(pitch_kd,init_quad_pitch_kd,dual_pitch_kd));
       
        }
        else if(arm_angle_degrees>47.0&&arm_angle_degrees<90.0)
        {
            float k_roll=cal_dynamic_roll_intertia(47.0);

            float k_roll_quad=J_roll_quad/(2*l_arm);
            float k_roll_dual=J_roll_dual/(2*h_upservo*0.173648);//sin(10)
            Kt_roll=fabsf(k_roll-k_roll_dual)/(fabsf(k_roll-k_roll_quad)+fabsf(k_roll-k_roll_dual));

            float dynamic_roll_kp=quad_roll_kp*Kt_roll+dual_roll_kp*(1-Kt_roll);
            float dynamic_roll_ki=quad_roll_ki*Kt_roll+dual_roll_ki*(1-Kt_roll);
            float dynamic_roll_kd=quad_roll_kd*Kt_roll+dual_roll_kd*(1-Kt_roll);


            float k_pitch=cal_dynamic_pitch_intertia(47.0);

            float k_pitch_quad=J_pitch_quad/(2*l_arm);
            float k_pitch_dual=J_pitch_dual/(4*l_arm);
            Kt_pitch=fabsf(k_pitch-k_pitch_dual)/(fabsf(k_pitch-k_pitch_quad)+fabsf(k_pitch-k_pitch_dual));
            
            float dynamic_pitch_kp=quad_pitch_kp*Kt_pitch+dual_pitch_kp*(1-Kt_pitch);
            float dynamic_pitch_ki=quad_pitch_ki*Kt_pitch+dual_pitch_ki*(1-Kt_pitch);
            float dynamic_pitch_kd=quad_pitch_kd*Kt_pitch+dual_pitch_kd*(1-Kt_pitch);
            //gcs().send_text(MAV_SEVERITY_NOTICE, "kp受影响%f",dynamic_pitch_kp);

            //roll
            get_rate_roll_pid().kP(constrain_float(dynamic_roll_kp,quad_roll_kp,dual_roll_kp));
            get_rate_roll_pid().kI(constrain_float(dynamic_roll_ki,quad_roll_ki,dual_roll_ki));
            get_rate_roll_pid().kD(constrain_float(dynamic_roll_kd,quad_roll_kd,dual_roll_kd));
            //pitch
            get_rate_pitch_pid().kP(constrain_float(dynamic_pitch_kp,quad_pitch_kp,dual_pitch_kp));
            get_rate_pitch_pid().kI(constrain_float(dynamic_pitch_ki,quad_pitch_ki,dual_pitch_ki));
            get_rate_pitch_pid().kD(constrain_float(dynamic_pitch_kd,quad_pitch_kd,dual_pitch_kd));
        }
        
        
        
    }
    */
    else if(rcarm_in<1900&&rcarm_in>1100)
    {
        arm_angle_degrees=get_arm_angle_degrees();

        Kt_roll=cal_dynamic_roll_trust_coefficient();
        gcs().send_text(MAV_SEVERITY_NOTICE, "滚转信任系数%f",Kt_roll);

        Kt_pitch=cal_dynamic_pitch_trust_coefficient();
        gcs().send_text(MAV_SEVERITY_NOTICE, "俯仰信任函数%f",Kt_pitch);

        if(arm_angle_degrees<90&&arm_angle_degrees>=23.0625)
        {
            float dynamic_roll_kp=init_quad_roll_kp*Kt_roll+init_dual_roll_kp*(1-Kt_roll);
            float dynamic_roll_ki=init_quad_roll_ki*Kt_roll+init_dual_roll_ki*(1-Kt_roll);
            float dynamic_roll_kd=init_quad_roll_kd*Kt_roll+init_dual_roll_kd*(1-Kt_roll);
       
            float dynamic_pitch_kp=init_quad_pitch_kp*Kt_pitch+init_dual_pitch_kp*(1-Kt_pitch);
            float dynamic_pitch_ki=init_quad_pitch_ki*Kt_pitch+init_dual_pitch_ki*(1-Kt_pitch);
            float dynamic_pitch_kd=init_quad_pitch_kd*Kt_pitch+init_dual_pitch_kd*(1-Kt_pitch);

            roll_kp=dynamic_roll_kp;
            roll_ki=dynamic_roll_ki;
            roll_kd=dynamic_roll_kd;

            pitch_kp=dynamic_pitch_kp;
            pitch_ki=dynamic_pitch_ki;
            pitch_kd=dynamic_pitch_kd;
        }
        else if(arm_angle_degrees<23.0625&&arm_angle_degrees>=10)
        {
            float dynamic_roll_kp=quad_roll_kp*Kt_roll+dual_roll_kp*(1-Kt_roll);
            float dynamic_roll_ki=quad_roll_ki*Kt_roll+dual_roll_ki*(1-Kt_roll);
            float dynamic_roll_kd=quad_roll_kd*Kt_roll+dual_roll_kd*(1-Kt_roll);
       
            float dynamic_pitch_kp=quad_pitch_kp*Kt_pitch+dual_pitch_kp*(1-Kt_pitch);
            float dynamic_pitch_ki=quad_pitch_ki*Kt_pitch+dual_pitch_ki*(1-Kt_pitch);
            float dynamic_pitch_kd=quad_pitch_kd*Kt_pitch+dual_pitch_kd*(1-Kt_pitch);

            roll_kp=dynamic_roll_kp;
            roll_ki=dynamic_roll_ki;
            roll_kd=dynamic_roll_kd;

            pitch_kp=dynamic_pitch_kp;
            pitch_ki=dynamic_pitch_ki;
            pitch_kd=dynamic_pitch_kd;
        }
        else if(arm_angle_degrees<10&&arm_angle_degrees>=0)
        {
            roll_kp=init_dual_roll_kp;
            roll_ki=init_dual_roll_ki;
            roll_kd=init_dual_roll_kd;

            pitch_kp=init_dual_pitch_kp;
            pitch_ki=init_dual_pitch_ki;
            pitch_kd=init_dual_pitch_kd;
        }
        
        //roll
        get_rate_roll_pid().kP(constrain_float(roll_kp, roll_kp_min,roll_kp_max));
        get_rate_roll_pid().kI(constrain_float(roll_ki, roll_ki_min,roll_ki_max));
        get_rate_roll_pid().kD(constrain_float(roll_kd, roll_kd_min,roll_kd_max));
        //pitch
        get_rate_pitch_pid().kP(constrain_float(pitch_kp, pitch_kp_min,pitch_kp_max));
        get_rate_pitch_pid().kI(constrain_float(pitch_ki, pitch_ki_min,pitch_ki_max));
        get_rate_pitch_pid().kD(constrain_float(pitch_kd, pitch_kd_min,pitch_kd_max));
    }

    else if(rcarm_in>=1900)
    {

        roll_kp=init_dual_roll_kp;
        roll_ki=init_dual_roll_ki;
        roll_kd=init_dual_roll_kd;

        pitch_kp=init_dual_pitch_kp;
        pitch_ki=init_dual_pitch_ki;
        pitch_kd=init_dual_pitch_kd;

        yaw_kp=0.18;
        yaw_ki=0.02;
        yaw_kd=0.00;

        //roll
        get_rate_roll_pid().kP(constrain_float(roll_kp, roll_kp_min,roll_kp_max));
        get_rate_roll_pid().kI(constrain_float(roll_ki, roll_ki_min,roll_ki_max));
        get_rate_roll_pid().kD(constrain_float(roll_kd, roll_kd_min,roll_kd_max));
        //pitch
        get_rate_pitch_pid().kP(constrain_float(pitch_kp, pitch_kp_min,pitch_kp_max));
        get_rate_pitch_pid().kI(constrain_float(pitch_ki, pitch_ki_min,pitch_ki_max));
        get_rate_pitch_pid().kD(constrain_float(pitch_kd, pitch_kd_min,pitch_kd_max));
        //yaw
        get_rate_yaw_pid().kP(constrain_float(yaw_kp,0.18,0.22));
        get_rate_yaw_pid().kI(constrain_float(yaw_ki,0.018,0.025));
        get_rate_yaw_pid().kI(constrain_float(yaw_kd,0.0,0.002));
    
    }
    
}

void AC_AttitudeControl_Multi::rate_controller_run()
{
    dynamic_pid();//动态调整pid的值

    // boost angle_p/pd each cycle on high throttle slew
    update_throttle_gain_boost();

    // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    update_throttle_rpy_mix();

    _ang_vel_body += _sysid_ang_vel_body;

    Vector3f gyro_latest = _ahrs.get_gyro_latest();

    _motors.set_roll(get_rate_roll_pid().update_all(_ang_vel_body.x, gyro_latest.x,  _dt, _motors.limit.roll, _pd_scale.x) + _actuator_sysid.x);
    _motors.set_roll_ff(get_rate_roll_pid().get_ff());

    _motors.set_pitch(get_rate_pitch_pid().update_all(_ang_vel_body.y, gyro_latest.y,  _dt, _motors.limit.pitch, _pd_scale.y) + _actuator_sysid.y);
    _motors.set_pitch_ff(get_rate_pitch_pid().get_ff());

    _motors.set_yaw(get_rate_yaw_pid().update_all(_ang_vel_body.z, gyro_latest.z,  _dt, _motors.limit.yaw, _pd_scale.z) + _actuator_sysid.z);
    _motors.set_yaw_ff(get_rate_yaw_pid().get_ff()*_feedforward_scalar);

    _sysid_ang_vel_body.zero();
    _actuator_sysid.zero();

    _pd_scale_used = _pd_scale;
    _pd_scale = VECTORF_111;

    control_monitor_update();
}

// sanity check parameters.  should be called once before takeoff
void AC_AttitudeControl_Multi::parameter_sanity_check()
{
    // sanity check throttle mix parameters
    if (_thr_mix_man < 0.1f || _thr_mix_man > AC_ATTITUDE_CONTROL_MAN_LIMIT) {
        // parameter description recommends thr-mix-man be no higher than 0.9 but we allow up to 4.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_man.set_and_save(constrain_float(_thr_mix_man, 0.1, AC_ATTITUDE_CONTROL_MAN_LIMIT));
    }
    if (_thr_mix_min < 0.1f || _thr_mix_min > AC_ATTITUDE_CONTROL_MIN_LIMIT) {
        _thr_mix_min.set_and_save(constrain_float(_thr_mix_min, 0.1, AC_ATTITUDE_CONTROL_MIN_LIMIT));
    }
    if (_thr_mix_max < 0.5f || _thr_mix_max > AC_ATTITUDE_CONTROL_MAX) {
        // parameter description recommends thr-mix-max be no higher than 0.9 but we allow up to 5.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_max.set_and_save(constrain_float(_thr_mix_max, 0.5, AC_ATTITUDE_CONTROL_MAX));
    }
    if (_thr_mix_min > _thr_mix_max) {
        _thr_mix_min.set_and_save(AC_ATTITUDE_CONTROL_MIN_DEFAULT);
        _thr_mix_max.set_and_save(AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
}

void AC_AttitudeControl_Multi::set_notch_sample_rate(float sample_rate)
{
#if AP_FILTER_ENABLED
    _pid_rate_roll.set_notch_sample_rate(sample_rate);
    _pid_rate_pitch.set_notch_sample_rate(sample_rate);
    _pid_rate_yaw.set_notch_sample_rate(sample_rate);
#endif
}
