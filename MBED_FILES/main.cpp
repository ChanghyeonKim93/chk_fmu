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
#define BAUD_RATE_USB 921600
#define BAUD_RATE_TELEMETRY 57600

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

#define ADC1_IN10 PC_0 // Analog Voltage Read
#define ADC1_IN11 PC_1
#define ADC1_IN12 PC_2
#define ADC1_IN13 PC_3

// Trigger
DigitalOut signal_trigger(PIN_TRIGGER);

// IMU
MPU9250 imu(SPI1_CS0, SPI1_MOSI, SPI1_MISO, SPI1_SCK, 
    AFS_8G, GFS_1000DPS, MFS_16BITS, MPU9250_FREQ_1000Hz);

InterruptIn imu_int(MPU9250_INT);

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
        us_curr      = timer.elapsed_time().count(); // Current time
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

// PWM outputs
DroneMotorPwm motor_pwm;
uint16_t pwm_values[8]; 
void setMotorPWM_01234567(uint16_t pwm_ushort[8]){
    for(int i = 0; i < 8; ++i){
        if(pwm_ushort[i] < 0 ) pwm_ushort[i] = 0;
        if(pwm_ushort[i] > 4095) pwm_ushort[i] = 4095;
    }
    motor_pwm.setPWM_all(pwm_ushort);
};

// Voltage reader                
USHORT_UNION vol_ushort[4];
AnalogIn voltage_adc0(ADC1_IN10);
AnalogIn voltage_adc1(ADC1_IN11);
AnalogIn voltage_adc2(ADC1_IN12);
AnalogIn voltage_adc3(ADC1_IN13);

void workfunction_getVoltages(){
    vol_ushort[0].ushort_ = voltage_adc0.read_u16(); // Read Analog voltage data (A0 pin, AnalogIn)
    vol_ushort[1].ushort_ = voltage_adc1.read_u16(); // Read Analog voltage data (A0 pin, AnalogIn)
    vol_ushort[2].ushort_ = voltage_adc2.read_u16(); // Read Analog voltage data (A0 pin, AnalogIn)
    vol_ushort[3].ushort_ = voltage_adc3.read_u16(); // Read Analog voltage data (A0 pin, AnalogIn)
};

// Serial
uint8_t packet_send[256];
uint8_t packet_recv[256];
uint32_t len_packet_recv = 0;
uint32_t len_packet_send = 0;
SerialCommunicatorMbed serial_usb(BAUD_RATE_USB, USART1_TX, USART1_RX);

uint8_t telemetry_send[256];
uint8_t telemetry_recv[256];
uint32_t len_telemetry_recv = 0;
uint32_t len_telemetry_send = 0;
SerialCommunicatorMbed serial_telemetry(BAUD_RATE_TELEMETRY, USART2_TX, USART2_RX);

void workfunction_readSerialUSB(){
    if(serial_usb.tryToReadSerialBuffer()) { // packet ready!
        int len_recv_message = 0;
        len_recv_message = serial_usb.getReceivedMessage(packet_recv); 

        if( len_recv_message == 16 ) { // Successfully received the packet.
            // In case of 8 PWM signals.
            // ======== USER-DEFINED CODE START ======== 

            USHORT_UNION pwm_tmp;
            pwm_tmp.bytes_[0] = packet_recv[0];  pwm_tmp.bytes_[1] = packet_recv[1];   pwm_values[0] = pwm_tmp.ushort_;
            pwm_tmp.bytes_[0] = packet_recv[2];  pwm_tmp.bytes_[1] = packet_recv[3];   pwm_values[1] = pwm_tmp.ushort_;
            pwm_tmp.bytes_[0] = packet_recv[4];  pwm_tmp.bytes_[1] = packet_recv[5];   pwm_values[2] = pwm_tmp.ushort_;
            pwm_tmp.bytes_[0] = packet_recv[6];  pwm_tmp.bytes_[1] = packet_recv[7];   pwm_values[3] = pwm_tmp.ushort_;
            pwm_tmp.bytes_[0] = packet_recv[8];  pwm_tmp.bytes_[1] = packet_recv[9];   pwm_values[4] = pwm_tmp.ushort_;
            pwm_tmp.bytes_[0] = packet_recv[10]; pwm_tmp.bytes_[1] = packet_recv[11];  pwm_values[5] = pwm_tmp.ushort_;
            pwm_tmp.bytes_[0] = packet_recv[12]; pwm_tmp.bytes_[1] = packet_recv[13];  pwm_values[6] = pwm_tmp.ushort_;
            pwm_tmp.bytes_[0] = packet_recv[14]; pwm_tmp.bytes_[1] = packet_recv[15];  pwm_values[7] = pwm_tmp.ushort_;

            setMotorPWM_01234567(pwm_values);     

            // ======== USER-DEFINED CODE END ========      
        }
    }
};
void workfunction_sendSerialUSB(){
    if(serial_usb.writable()){
        serial_usb.send_withChecksum(packet_send, len_packet_send);
        len_packet_send = 0;
    }
};

void workfunction_readSerialTelemetry(){
    if(serial_telemetry.tryToReadSerialBuffer()) { // packet ready!
        int len_recv_message = 0;
        len_recv_message = serial_telemetry.getReceivedMessage(telemetry_recv); 

        if( len_recv_message > 0 ) { // Successfully received the packet.
            // ======== USER-DEFINED CODE START ======== 

            // do your code.

            // ======== USER-DEFINED CODE END ========      
        }
    }
};
void workfunction_sendSerialTelemetry(){
    if(serial_telemetry.writable()){
        serial_telemetry.send_withChecksum(telemetry_send, len_telemetry_send);
        len_telemetry_send = 0;
    }
};

void tryToReadSerialUSB(){ event_queue.call(workfunction_readSerialUSB); };
void tryToSendSerialUSB(){ event_queue.call(workfunction_sendSerialUSB); };
void tryToReadSerialTelemetry(){ event_queue.call(workfunction_readSerialTelemetry); };
void tryToSendSerialTelemetry(){ event_queue.call(workfunction_sendSerialTelemetry); };

int main() {
    signal_trigger = 0;

    // Timer starts.
    timer.start();
    std::chrono::microseconds time_send_usb_prev = timer.elapsed_time();
    std::chrono::microseconds time_recv_usb_prev = timer.elapsed_time();
    
    std::chrono::microseconds time_send_telemetry_prev = timer.elapsed_time();
    std::chrono::microseconds time_recv_telemetry_prev = timer.elapsed_time();

    std::chrono::microseconds time_curr;

    pwm_values[0] = 0;
    pwm_values[1] = 0;
    pwm_values[2] = 0;
    pwm_values[3] = 0;
    pwm_values[4] = 0;
    pwm_values[5] = 0;
    pwm_values[6] = 0;
    pwm_values[7] = 0;

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
    
    signal_trigger = 1;
    ThisThread::sleep_for(200ms);
    signal_trigger = 0;
    ThisThread::sleep_for(200ms);
    signal_trigger = 1;
    ThisThread::sleep_for(200ms);
    signal_trigger = 0;
    ThisThread::sleep_for(200ms);
    signal_trigger = 1;
    ThisThread::sleep_for(200ms);
    signal_trigger = 0;
    ThisThread::sleep_for(200ms);
    signal_trigger = 1;
    ThisThread::sleep_for(200ms);
    signal_trigger = 0;
    ThisThread::sleep_for(200ms);
    signal_trigger = 1;
    ThisThread::sleep_for(200ms);
    signal_trigger = 0;


    flag_IMU_init = true;

    int cnt = 0;
    int ii  = 0;

    USHORT_UNION telemetry_send_count;
    telemetry_send_count.ushort_ = 0;
    while (true) {
        // Write if writable.
        time_curr = timer.elapsed_time();
        std::chrono::duration<int, std::micro> dt_send_usb = time_curr - time_send_usb_prev;
        std::chrono::duration<int, std::micro> dt_send_telemetry = time_curr - time_send_telemetry_prev;

        // Telemetry send
        // if(dt_send_telemetry.count() > 499999) {
        //     ++telemetry_send_count.ushort_;

        //     telemetry_send[0]  = telemetry_send_count.bytes_[0];
        //     telemetry_send[1]  = telemetry_send_count.bytes_[1];
        //     telemetry_send[2]  = 'c';
        //     telemetry_send[3]  = 'd';
        //     telemetry_send[4]  = 'a';
        //     telemetry_send[5]  = 'b';
        //     telemetry_send[6]  = 'c';
        //     telemetry_send[7]  = 'd';

        //     len_telemetry_send = 8;
        //     if(serial_telemetry.writable()) 
        //         tryToSendSerialTelemetry();
            
        //     time_send_telemetry_prev = time_curr;
        // }

        // IMU received.
        if(flag_imu_ready){
            
            if(serial_usb.writable()) { // If serial USB can be written,
                // Trigger signal output.
                ++trigger_count;
                if(trigger_count >= trigger_step){
                    trigger_count = 0;
                    flag_camera_trigger = CAMERA_TRIGGER_HIGH;
                    signal_trigger = 1;
                }
                
                // IMU data (3D acc, 3D gyro, 3D magnetometer)
                packet_send[0]  = acc_short[0].bytes_[0]; packet_send[1]  = acc_short[0].bytes_[1];
                packet_send[2]  = acc_short[1].bytes_[0]; packet_send[3]  = acc_short[1].bytes_[1];
                packet_send[4]  = acc_short[2].bytes_[0]; packet_send[5]  = acc_short[2].bytes_[1];
                
                packet_send[6]  = gyro_short[0].bytes_[0]; packet_send[7]  = gyro_short[0].bytes_[1];
                packet_send[8]  = gyro_short[1].bytes_[0]; packet_send[9]  = gyro_short[1].bytes_[1];
                packet_send[10] = gyro_short[2].bytes_[0]; packet_send[11] = gyro_short[2].bytes_[1];

                packet_send[12] = mag_short[0].bytes_[0]; packet_send[13] = mag_short[0].bytes_[1];
                packet_send[14] = mag_short[1].bytes_[0]; packet_send[15] = mag_short[1].bytes_[1];
                packet_send[16] = mag_short[2].bytes_[0]; packet_send[17] = mag_short[2].bytes_[1];

                packet_send[18]  = tsec.bytes_[0];  // time (second part, low)
                packet_send[19]  = tsec.bytes_[1];  // time (second part, high)
                
                packet_send[20]  = tusec.bytes_[0]; // time (microsecond part, lowest)
                packet_send[21]  = tusec.bytes_[1]; // time (microsecond part, low)
                packet_send[22]  = tusec.bytes_[2]; // time (microsecond part, high)
                packet_send[23]  = tusec.bytes_[3]; // time (microsecond part, highest)

                // Camera trigger state
                packet_send[24]  = flag_camera_trigger; // 'CAMERA_TRIGGER_HIGH' or 'CAMERA_TRIGGER_LOW'

                // AnalogIn (battery voltage data)
                workfunction_getVoltages();
                packet_send[25]  = vol_ushort[0].bytes_[0];
                packet_send[26]  = vol_ushort[0].bytes_[1];
                packet_send[27]  = vol_ushort[1].bytes_[0];
                packet_send[28]  = vol_ushort[1].bytes_[1];
                packet_send[29]  = vol_ushort[2].bytes_[0];
                packet_send[30]  = vol_ushort[2].bytes_[1];
                packet_send[31]  = vol_ushort[3].bytes_[0];
                packet_send[32]  = vol_ushort[3].bytes_[1];

                // Initialize flags
                signal_trigger = 0;
                flag_camera_trigger = CAMERA_TRIGGER_LOW;

                // Send length
                len_packet_send = 33;

                tryToSendSerialUSB(); // Send!

                flag_imu_ready = false;
                time_send_usb_prev = time_curr;
            }
        }            
    
        // Read data if data exists.
        if(serial_usb.readable()) {
            tryToReadSerialUSB();
        }
        
    }
    
    return 0;
}
