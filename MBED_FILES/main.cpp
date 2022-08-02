#include "ThisThread.h"
#include "mbed.h"

#include "BufferedSerial.h"
#include "platform/mbed_thread.h"
#include "serial_comm_mbed.h"
#include "drone_motor_pwm.h"

#include "mpu9250_spi.h"
#include "union_struct.h"
#include <chrono>

// Timer to know how much time elapses.
Timer timer;

// Set event queue and worker thread
Thread thread_poll(osPriorityRealtime); // polling all interrupt signals. https://os.mbed.com/docs/mbed-os/v6.10/apis/thread.html
EventQueue event_queue(128 * EVENTS_EVENT_SIZE);

// Parameter settings
#define BAUD_RATE 921600

#define MPU9250_INT PE_2
#define PIN_TRIGGER PA_15

#define CAN1_TX PA_12
#define CAN1_RX PA_11

#define USART1_RX PA_10
#define USART1_TX PA_9

#define USART2_TX PD_5
#define USART2_RX PD_6

#define TIM2_CH1 PA_0
#define TIM2_CH2 PA_1
#define TIM2_CH3 PA_2
#define TIM2_CH4 PA_3

#define TIM3_CH1 PC_6
#define TIM3_CH2 PC_7
#define TIM3_CH3 PB_0
#define TIM3_CH4 PB_1

#define TIM4_CH1 PD_12
#define TIM4_CH2 PD_13
#define TIM4_CH3 PD_14
#define TIM4_CH4 PD_15

#define I2C1_SCL PB_6
#define I2C1_SDA PB_7

#define SPI1_CS0  PA_4
#define SPI1_SCK  PA_5
#define SPI1_MISO PA_6
#define SPI1_MOSI PA_7

#define SPI2_CS0  PB_12
#define SPI2_SCK  PB_13
#define SPI2_MISO PB_14
#define SPI2_MOSI PB_15

#define ADC1_IN10 PC_0
#define ADC1_IN11 PC_1
#define ADC1_IN12 PC_2
#define ADC1_IN13 PC_3

// IMU
MPU9250 imu(SPI1_CS0, SPI1_MOSI, SPI1_MISO, SPI1_SCK, 
    AFS_8G, GFS_1000DPS, MFS_16BITS, MPU9250_FREQ_1000Hz);

InterruptIn imu_int(MPU9250_INT);
DigitalOut signal_trigger(PIN_TRIGGER);

int16_t data[9];
SHORT_UNION acc_short[3];
SHORT_UNION gyro_short[3];
SHORT_UNION mag_short[3];

uint64_t us_curr;
USHORT_UNION tsec;
UINT_UNION   tusec;

#define CAMERA_TRIGGER_LOW  0b01010101
#define CAMERA_TRIGGER_HIGH 0b10101010

volatile bool flag_IMU_init  = false;
volatile bool flag_imu_ready = false;

volatile uint8_t trigger_step  = 25;
volatile uint8_t trigger_count = 0;
volatile uint8_t flag_camera_trigger = CAMERA_TRIGGER_LOW;

void workfunction_getIMU(){
    if(flag_IMU_init){
        // Current time
        us_curr      = timer.elapsed_time().count();
        tsec.ushort_ = (uint16_t)(us_curr/1000000);
        tusec.uint_  = (uint32_t)(us_curr-((uint32_t)tsec.ushort_)*1000000);
        
        imu.read9AxisRaw(data);
        acc_short[0].ushort_ = data[0];
        acc_short[1].ushort_ = data[1];
        acc_short[2].ushort_ = data[2];

        gyro_short[0].ushort_ = data[3];
        gyro_short[1].ushort_ = data[4];
        gyro_short[2].ushort_ = data[5];

        mag_short[0].ushort_ = data[6];
        mag_short[1].ushort_ = data[7];
        mag_short[2].ushort_ = data[8];

        flag_imu_ready = true;
    }
};
void ISR_IMU(){
    event_queue.call(workfunction_getIMU);
};


// Serial
uint8_t packet_send[256];
uint8_t packet_recv[256];

uint32_t len_packet_recv = 0;
uint32_t len_packet_send = 0;
SerialCommunicatorMbed serial(BAUD_RATE);

DroneMotorPwm motor_pwm;
uint16_t pwm_values[8]; 

void setMotorPWM(uint8_t n, uint16_t pwm_ushort){
    if(pwm_ushort > 4095) pwm_ushort = 4095;
    if(pwm_ushort < 0) pwm_ushort = 0;
    motor_pwm.setPWM(n, pwm_ushort);
};

void setMotorPWM_01234567(uint16_t pwm_ushort[8]){
    for(int i = 0; i < 8; ++i){
        if(pwm_ushort[i] < 0 ) pwm_ushort[i] = 0;
        if(pwm_ushort[i] > 4095) pwm_ushort[i] = 4095;
    }
    motor_pwm.setPWM_all(pwm_ushort);
};

void workfunction_readSerial(){
    if(serial.tryToReadSerialBuffer()) { // packet ready!
        int len_recv_message = 0;
        len_recv_message = serial.getReceivedMessage(packet_recv); 

        if( len_recv_message > 0 ) { // Successfully received the packet.
            // do something...
        }
    }
};
void tryToReadSerial(){
    event_queue.call(workfunction_readSerial);
};

void workfunction_sendSerial(){
    if(serial.writable()){
        serial.send_withChecksum(packet_send, len_packet_send);
        len_packet_send = 0;
    }
}

void tryToSendSerial(){
    event_queue.call(workfunction_sendSerial);
};

int main() {
    // Timer starts.
    timer.start();
    std::chrono::microseconds time_send_prev = timer.elapsed_time();
    std::chrono::microseconds time_recv_prev = timer.elapsed_time();
    std::chrono::microseconds time_curr;

    pwm_values[0] = 500;
    pwm_values[1] = 2047;
    pwm_values[2] = 500;
    pwm_values[3] = 2047;
    pwm_values[4] = 500;
    pwm_values[5] = 2047;
    pwm_values[6] = 500;
    pwm_values[7] = 2047;

    // Start the event queue
    thread_poll.start(callback(&event_queue, &EventQueue::dispatch_forever));

    // Start the MPU-9250 
    imu.resetMPU9250();
    ThisThread::sleep_for(200ms);

    float gyroBias[3]       = {0, 0, 0};
    float accelBias[3]      = {0, 0, 0}; // Bias corrections for gyro and accelerometer
    float magCalibration[3] = {0, 0, 0};

    imu.calibrateMPU9250(gyroBias, accelBias);
    ThisThread::sleep_for(200ms);

    imu.initMPU9250();
    ThisThread::sleep_for(200ms);
    
    imu.initAK8963(magCalibration);
    ThisThread::sleep_for(200ms);
    
    // Start interrupt at rising edge.
    imu_int.rise(ISR_IMU);

    // signal_trigger low.
    signal_trigger = 0;
                // setMotorPWM_01234567(pwm_values);

    flag_IMU_init = true;
    while (true) {
        // Write if writable.
        time_curr = timer.elapsed_time();
        std::chrono::duration<int, std::micro> dt_send = time_curr - time_send_prev;
        // time_curr.count();

        if(flag_imu_ready){
            setMotorPWM_01234567(pwm_values);
            if(serial.writable()) {

                for(int ii= 0; ii < 4; ++ii){
                    unsigned short tmp0 = pwm_values[2*ii];
                    pwm_values[2*ii] = pwm_values[2*ii+1];
                    pwm_values[2*ii+1] = tmp0;
                }

                ++trigger_count;
                if(trigger_count >= trigger_step){
                    trigger_count = 0;
                    flag_camera_trigger = CAMERA_TRIGGER_HIGH;
                    signal_trigger = 1;
                }
                
                packet_send[0]  = acc_short[0].bytes_[0];
                packet_send[1]  = acc_short[0].bytes_[1];
                packet_send[2]  = acc_short[1].bytes_[0];
                packet_send[3]  = acc_short[1].bytes_[1];
                packet_send[4]  = acc_short[2].bytes_[0];
                packet_send[5]  = acc_short[2].bytes_[1];
                
                packet_send[6]  = gyro_short[0].bytes_[0];
                packet_send[7]  = gyro_short[0].bytes_[1];
                packet_send[8]  = gyro_short[1].bytes_[0];
                packet_send[9]  = gyro_short[1].bytes_[1];
                packet_send[10] = gyro_short[2].bytes_[0];
                packet_send[11] = gyro_short[2].bytes_[1];

                packet_send[12] = mag_short[0].bytes_[0];
                packet_send[13] = mag_short[0].bytes_[1];
                packet_send[14] = mag_short[1].bytes_[0];
                packet_send[15] = mag_short[1].bytes_[1];
                packet_send[16] = mag_short[2].bytes_[0];
                packet_send[17] = mag_short[2].bytes_[1];

                packet_send[18]  = tsec.bytes_[0];  // time (second part, low)
                packet_send[19]  = tsec.bytes_[1];  // time (second part, high)
                
                packet_send[20]  = tusec.bytes_[0]; // time (microsecond part, lowest)
                packet_send[21]  = tusec.bytes_[1]; // time (microsecond part, low)
                packet_send[22]  = tusec.bytes_[2]; // time (microsecond part, high)
                packet_send[23]  = tusec.bytes_[3]; // time (microsecond part, highest)

                packet_send[24]  = flag_camera_trigger; // 'CAMERA_TRIGGER_HIGH' or 'CAMERA_TRIGGER_LOW'

                len_packet_send = 25;
                tryToSendSerial();

                flag_imu_ready = false;

                signal_trigger = 0;
                flag_camera_trigger = CAMERA_TRIGGER_LOW;
            }
            time_send_prev = time_curr;
        }            
    
        // Read data if data exists.
        if(serial.readable()) {
            tryToReadSerial();
        }
        
    }
    
    return 0;
}