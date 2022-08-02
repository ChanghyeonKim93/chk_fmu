#ifndef _DRONE_MOTOR_PWM_H_
#define _DRONE_MOTOR_PWM_H_
#include "mbed.h"

#define NUM_MOTORS 8

// PWM (TIMER 3)
#define MOTOR_0_PWM PA_0  // pwm2/1
#define MOTOR_1_PWM PA_1  // pwm2/2
#define MOTOR_2_PWM PA_2 // pwm2/3
#define MOTOR_3_PWM PA_3  // pwm2/4

// PWM (TIMER 4)
#define MOTOR_4_PWM PC_6  // pwm3/1
#define MOTOR_5_PWM PC_7  // pwm3/2
#define MOTOR_6_PWM PB_0  // pwm3/3
#define MOTOR_7_PWM PB_1  // pwm3/4

#define PWM_FREQUENCY 3000 // Hz

class DroneMotorPwm {
private:
    PwmOut pwm_[8];

public:
    DroneMotorPwm();

    void setPWM_all(uint16_t duty[NUM_MOTORS]);
    void setPWM(uint32_t num, uint16_t duty);

};


#endif