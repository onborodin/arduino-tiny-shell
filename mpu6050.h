
/*
MPU6050 lib 0x02

copyright (c) Davide Gironi, 2012

Released under GPLv3.
Please refer to LICENSE file for licensing information.

References:
  - most of the code is a port of the arduino mpu6050 library by Jeff Rowberg
    https://github.com/jrowberg/i2cdevlib
  - Mahony complementary filter for attitude estimation
    http://www.x-io.co.uk
*/


#ifndef MPU6050_H_
#define MPU6050_H_

#include <avr/io.h>
#include <mpu6050reg.h>


/* definitions */
#define MPU6050_ADDR (0x68 << 1) /* device address - 0x68 pin low (GND), 0x69 pin high (VCC) */

/* Enable the getattitude functions */
/* Because we do not have a magnetometer, we have to start the chip always in the same position */
/* then to obtain your object attitude you have to apply the aerospace sequence */
/* 0 disabled */
/* 1 mahony filter */
/* 2 dmp chip processor */
#ifndef MPU6050_GETATTITUDE
#define MPU6050_GETATTITUDE 2
#endif

/* Definitions for raw data  gyro and acc scale */
#define MPU6050_GYRO_FS         MPU6050_GYRO_FS_2000
#define MPU6050_ACCEL_FS        MPU6050_ACCEL_FS_2

#define MPU6050_GYRO_LSB_250    131.0
#define MPU6050_GYRO_LSB_500    65.5
#define MPU6050_GYRO_LSB_1000   32.8
#define MPU6050_GYRO_LSB_2000   16.4

#if MPU6050_GYRO_FS == MPU6050_GYRO_FS_250
    #define MPU6050_GGAIN   MPU6050_GYRO_LSB_250
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_500
    #define MPU6050_GGAIN   MPU6050_GYRO_LSB_500
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_1000
    #define MPU6050_GGAIN   MPU6050_GYRO_LSB_1000
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_2000
    #define MPU6050_GGAIN   MPU6050_GYRO_LSB_2000
#endif

#define MPU6050_ACCEL_LSB_2     16384.0
#define MPU6050_ACCEL_LSB_4      8192.0
#define MPU6050_ACCEL_LSB_8      4096.0
#define MPU6050_ACCEL_LSB_16     2048.0

#if MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_2
    #define MPU6050_AGAIN MPU6050_ACCEL_LSB_2
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_4
    #define MPU6050_AGAIN MPU6050_ACCEL_LSB_4
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_8
    #define MPU6050_AGAIN MPU6050_ACCEL_LSB_8
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_16
    #define MPU6050_AGAIN MPU6050_ACCEL_LSB_16
#endif

#define MPU6050_CALIBRATEDACCGYRO 0     /* Set to 1 if is calibrated */

#if MPU6050_CALIBRATEDACCGYRO == 1
#define MPU6050_AXOFFSET        0
#define MPU6050_AYOFFSET        0
#define MPU6050_AZOFFSET        0

#define MPU6050_AXGAIN          16384.0
#define MPU6050_AYGAIN          16384.0
#define MPU6050_AZGAIN          16384.0

#define MPU6050_GXOFFSET        -42
#define MPU6050_GYOFFSET        9
#define MPU6050_GZOFFSET        -29

#define MPU6050_GXGAIN          16.4
#define MPU6050_GYGAIN          16.4
#define MPU6050_GZGAIN          16.4
#endif

/* Definitions for attitude 1 function estimation */
#if MPU6050_GETATTITUDE == 1
/*
  Setup timer0 overflow event and define madgwickAHRSsampleFreq equal to timer0 frequency
  timerfreq = (FCPU / prescaler) / timerscale
       timerscale 8-bit = 256
  es. 61 = (16000000 / 1024) / 256 
 */
#define MPU6050_TIMER0INIT              TCCR0B |=(1<<CS02)|(1<<CS00); TIMSK0 |=(1<<TOIE0);
#define MPU6050_MAHONY_SAMPLE_FREQ      61.0f  /*  Sample frequency in Hz */
#define MPU6050_MAHONY_TWO_KP_DEF       (2.0f * 0.5f)    /*  2 * proportional gain */
#define MPU6050_MAHONY_TWO_KI_DEF       (2.0f * 0.1f)    /*  2 * integral gain */

#endif

#if MPU6050_GETATTITUDE == 2

/* DMP definitions */
/* Packet size */
#define MPU6050_DMP_PACKET_SIZE 42
/* Define INT0 rise edge interrupt */
#define MPU6050_DMP_INT0SETUP   EICRA |= (1<<ISC01) | (1<<ISC00)
/* Define enable and disable INT0 rise edge interrupt */
#define MPU6050_DMP_INT0DISABLE (EIMSK &= ~(1<<INT0))
#define MPU6050_DMP_INT0ENABLE  (EIMSK |= (1<<INT0))
extern volatile uint8_t mpu6050_mpu_interrupt;

#endif

/* Functions */
void mpu6050_init(void);
uint8_t mpu6050_test_connection(void);

//#if MPU6050_GETATTITUDE == 0
void mpu6050_get_raw_data(int16_t * ax, int16_t * ay, int16_t * az, int16_t * gx, int16_t * gy, int16_t * gz);
void mpu6050_get_conv_data(double *axg, double *ayg, double *azg, double *gxds, double *gyds, double *gzds);
//#endif

void mpu6050_set_sleep_disabled(void);
void mpu6050_set_sleep_enabled(void);

int8_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t length, uint8_t * data);
int8_t mpu6050_read_byte(uint8_t reg_addr, uint8_t * data);
void mpu6050_write_bytes(uint8_t reg_addr, uint8_t length, uint8_t * data);
void mpu6050_write_byte(uint8_t reg_addr, uint8_t data);
int8_t mpu6050_read_bits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t * data);
int8_t mpu6050_read_bit(uint8_t reg_addr, uint8_t bit_num, uint8_t * data);
void mpu6050_write_bits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data);
void mpu6050_write_bit(uint8_t reg_addr, uint8_t bit_num, uint8_t data);

#if MPU6050_GETATTITUDE == 1
void mpu6050_update_quaternion(void);
void mpu6050_get_quaternion(double *qw, double *qx, double *qy, double *qz);
void mpu6050_get_roll_pitch_yaw(double *pitch, double *roll, double *yaw);

void mpu6050_mahony_update(float gx, float gy, float gz, float ax, float ay, float az);
#endif

#if MPU6050_GETATTITUDE == 2
void mpu6050_write_words(uint8_t reg_addr, uint8_t length, uint16_t * data);
void mpu6050_set_memory_bank(uint8_t bank, uint8_t prefetch_enabled, uint8_t user_bank);
void mpu6050_set_memory_start_address(uint8_t address);
void mpu6050_read_memory_block(uint8_t * data, uint16_t data_size, uint8_t bank, uint8_t address);
uint8_t mpu6050_write_memory_block(const uint8_t * data, uint16_t data_size, uint8_t bank, uint8_t address,
                                        uint8_t verify, uint8_t use_prog_mem);
uint8_t mpu6050_write_dmp_configuration_set(const uint8_t * data, uint16_t data_size, uint8_t use_prog_mem);
uint16_t mpu6050_get_fifo_count();
void mpu6050_get_fifo_bytes(uint8_t * data, uint8_t length);

uint8_t mpu6050_get_int_status(void);
void mpu6050_reset_fifo(void);

int8_t mpu6050_get_X_gyro_offset(void);
int8_t mpu6050_get_Y_gyro_offset(void);
int8_t mpu6050_get_Z_gyro_offset(void);

void mpu6050_set_X_gyro_offset(int8_t offset);
void mpu6050_set_Y_gyro_offset(int8_t offset);
void mpu6050_set_Z_gyro_offset(int8_t offset);

/* Base DMP */
uint8_t mpu6050_dmp_initialize(void);
void mpu6050_dmp_enable(void);
void mpu6050_dmp_disable(void);
void mpu6050_get_quaternion(const uint8_t * packet, double *qw, double *qx, double *qy, double *qz);
void mpu6050_get_roll_pitch_yaw(double qw, double qx, double qy, double qz, double *roll, double *pitch,
                                    double *yaw);
uint8_t mpu6050_get_quaternion_wait(double *qw, double *qx, double *qy, double *qz);
#endif

#endif
