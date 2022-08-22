#ifndef _DRONE_MOTOR_PWM_H_
#define _DRONE_MOTOR_PWM_H_
#include "mbed.h"

#define NUM_MOTORS 12

// PWM (TIMER 2)
#define MOTOR_0_PWM PA_0  // pwm2/1
#define MOTOR_1_PWM PA_1  // pwm2/2
#define MOTOR_2_PWM PA_2 // pwm2/3
#define MOTOR_3_PWM PA_3  // pwm2/4

// PWM (TIMER 3)
#define MOTOR_4_PWM PC_6  // pwm3/1
#define MOTOR_5_PWM PC_7  // pwm3/2
#define MOTOR_6_PWM PB_0  // pwm3/3
#define MOTOR_7_PWM PB_1  // pwm3/4

// PWM (TIMER 4)
#define MOTOR_8_PWM  PD_12  // pwm4/1
#define MOTOR_9_PWM  PD_13  // pwm4/2
#define MOTOR_10_PWM PD_14  // pwm4/3
#define MOTOR_11_PWM PD_15  // pwm4/4

#define PWM_FREQUENCY 500 // Hz

class DroneMotorPwm {
private:
    PwmOut pwm_[NUM_MOTORS];

public:
    DroneMotorPwm();

    void setPWM_all(uint16_t duty[NUM_MOTORS]);
    void setPWM(uint32_t num, uint16_t duty);

};


#endif