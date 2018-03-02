
/*
  Original:MPU6050 lib 0x02, copyright (c) Davide Gironi, 2012
  Copyright (c) Oleg Borodin, 2018
  Code updated from http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 */

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <mpu6050.h>

#if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2
#include <math.h>
#endif

#include <twim.h>

volatile uint8_t buffer[14];

/* Read bytes from chip register */
int8_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t length, uint8_t * data) {
    uint8_t i = 0;
    int8_t count = 0;

    if (length > 0) {
        /* Request register */
        i2c_start(MPU6050_ADDR | I2C_WRITE);
        i2c_write(reg_addr);
        _delay_us(10);
        /* Read data */
        i2c_start(MPU6050_ADDR | I2C_READ);
        for (i = 0; i < length; i++) {
            count++;
            if (i == length - 1)
                data[i] = i2c_readNak();
            else
                data[i] = i2c_readAck();
        }
        i2c_stop();
    }
    return count;
}

/* Read 1 byte from chip register */
int8_t mpu6050_read_byte(uint8_t reg_addr, uint8_t * data) {
    return mpu6050_read_bytes(reg_addr, 1, data);
}

/* Write bytes to chip register */
void mpu6050_write_bytes(uint8_t reg_addr, uint8_t length, uint8_t * data) {
    if (length > 0) {
        /* Write data */
        i2c_start(MPU6050_ADDR | I2C_WRITE);
        i2c_write(reg_addr);

        for (uint8_t i = 0; i < length; i++) {
            i2c_write((uint8_t) data[i]);
        }
        i2c_stop();
    }
}

/* Write 1 byte to chip register */
void mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
    return mpu6050_write_bytes(reg_addr, 1, &data);
}

/* Read bits from chip register */
int8_t mpu6050_read_bits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t * data) {
    /* 01101001 read byte */
    /* 76543210 bit numbers */
    /*    xxx   args: bit_start=4, length=3 */
    /*    010   masked */
    /*   -> 010 shifted */
    int8_t count = 0;
    if (length > 0) {
        uint8_t b;
        if ((count = mpu6050_read_byte(reg_addr, &b)) != 0) {
            uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
            b &= mask;
            b >>= (bit_start - length + 1);
            *data = b;
        }
    }
    return count;
}

/* Read 1 bit from chip register */
int8_t mpu6050_read_bit(uint8_t reg_addr, uint8_t bit_num, uint8_t * data) {
    uint8_t b;
    uint8_t count = mpu6050_read_byte(reg_addr, &b);
    *data = b & (1 << bit_num);
    return count;
}

/* Write bit/bits to chip register */
void mpu6050_write_bits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data) {
    /*      010 value to write */
    /* 76543210 bit numbers */
    /*    xxx   args: bit_start=4, length=3 */
    /* 00011100 mask byte */
    /* 10101111 original value (sample) */
    /* 10100011 original & ~mask */
    /* 10101011 masked | value */
    if (length > 0) {
        uint8_t b = 0;
        if (mpu6050_read_byte(reg_addr, &b) != 0) {     /* get current data */
            uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
            data <<= (bit_start - length + 1);  /* shift data into correct position */
            data &= mask;       /* zero all non-important bits in data */
            b &= ~(mask);       /* zero all important bits in existing byte */
            b |= data;          /* combine data with existing byte */
            mpu6050_write_byte(reg_addr, b);
        }
    }
}

/* Write one bit to chip register */
void mpu6050_write_bit(uint8_t reg_addr, uint8_t bit_num, uint8_t data) {
    uint8_t b;
    mpu6050_read_byte(reg_addr, &b);
    b = (data != 0) ? (b | (1 << bit_num)) : (b & ~(1 << bit_num));
    mpu6050_write_byte(reg_addr, b);
}

/* Set sleep disabled */
void mpu6050_set_sleep_disabled() {
    mpu6050_write_bit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0);
}

/* Set sleep enabled */
void mpu6050_set_sleep_enabled() {
    mpu6050_write_bit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 1);
}


/* Test connectino to chip */
uint8_t mpu6050_test_connection(void) {
    volatile uint8_t buffer[1];
    mpu6050_read_bits(MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, (uint8_t *) buffer);
    if (buffer[0] == 0x34)
        return 1;
    else
        return 0;
}

/* Initialize the accel and gyro */
void mpu6050_init() {

    /* Allow mpu6050 chip clocks to start up */
    _delay_ms(100);

    /* Aet sleep disabled */
    mpu6050_set_sleep_disabled();
    /* Wake up delay needed sleep disabled */
    _delay_ms(10);

    /*
       Set clock source
       It is highly recommended that the device be configured to use one of the gyroscopes (or an external clock source)
       as the clock reference for improved stability
     */
    mpu6050_write_bits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
    /* Set DLPF bandwidth to 42Hz */
    mpu6050_write_bits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_42);
    /* Set sampe rate */
    mpu6050_write_byte(MPU6050_RA_SMPLRT_DIV, 4);       /* 1khz / (1 + 4) = 200Hz */
    /* Set gyro range */
    mpu6050_write_bits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS);
    /* Set accel range */
    mpu6050_write_bits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS);

#if MPU6050_GETATTITUDE == 1
    /* MPU6050_TIMER0INIT */
#endif
}

/* Can not accept many request if we alreay have getattitude requests */

/* Get raw data */
void mpu6050_get_raw_data(int16_t * ax, int16_t * ay, int16_t * az, int16_t * gx, int16_t * gy, int16_t * gz) {
    volatile uint8_t buffer[14];
    mpu6050_read_bytes(MPU6050_RA_ACCEL_XOUT_H, 14, (uint8_t *) buffer);

    *ax = (((int16_t) buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t) buffer[2]) << 8) | buffer[3];
    *az = (((int16_t) buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t) buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t) buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t) buffer[12]) << 8) | buffer[13];
}

/* Get raw data converted to g and deg/sec values */
void mpu6050_get_conv_data(double *axg, double *ayg, double *azg, double *gxds, double *gyds, double *gzds) {
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
    mpu6050_get_raw_data(&ax, &ay, &az, &gx, &gy, &gz);

#if MPU6050_CALIBRATEDACCGYRO == 1
    *axg = (double)(ax - MPU6050_AXOFFSET) / MPU6050_AXGAIN;
    *ayg = (double)(ay - MPU6050_AYOFFSET) / MPU6050_AYGAIN;
    *azg = (double)(az - MPU6050_AZOFFSET) / MPU6050_AZGAIN;

    *gxds = (double)(gx - MPU6050_GXOFFSET) / MPU6050_GXGAIN;
    *gyds = (double)(gy - MPU6050_GYOFFSET) / MPU6050_GYGAIN;
    *gzds = (double)(gz - MPU6050_GZOFFSET) / MPU6050_GZGAIN;
#else
    *axg = (double)(ax) / MPU6050_AGAIN;
    *ayg = (double)(ay) / MPU6050_AGAIN;
    *azg = (double)(az) / MPU6050_AGAIN;

    *gxds = (double)(gx) / MPU6050_GGAIN;
    *gyds = (double)(gy) / MPU6050_GGAIN;
    *gzds = (double)(gz) / MPU6050_GGAIN;
#endif
}


/*
  Quaternion of sensor frame relative to auxiliary frame:
    quaternion_t q = {
        .q0 = 1.0f, .q1 = 0.0f, .q2 = 0.0f, .q3 = 0.0f,
        .integralFBx = 0.0f, .integralFBy = 0.0f, .integralFBz = 0.0f
    };

    quaternion_t *qn = &q;

  Update timer for attitude:
    ISR(TIMER0_OVF_vect) {
        mpu6050_update_quaternion(qn);
    }

  Setup timer0 overflow event and define madgwickAHRSsampleFreq equal to timer0 frequency
  timerfreq = (FCPU / prescaler) / timerscale
       timerscale 8-bit = 256
  es. 61 = (16000000 / 1024) / 256

    void timer_init(void) {
        TCCR0B |=(1<<CS02)|(1<<CS00);
        TIMSK0 |=(1<<TOIE0);
    }

  Main 
    main(void) {
        ...
        double roll, pitch, yaw;
        uint8_t roll_str[8], pitch_str[8], yaw_str[8];

        mpu6050_get_roll_pitch_yaw(qn, &roll, &pitch, &yaw);

        snprintf(roll_str, 6, "%+5.0f", roll * (240/M_PI)/1.27);
        snprintf(pitch_str, 6, "%+5.0f", pitch * (240/M_PI)/1.27);
        snprintf(yaw_str, 6, "%+5.0f", yaw * (240/M_PI)/1.27);
        ...
    }
 */


/* Update quaternion */
void mpu6050_update_quaternion(quaternion_t *qn) {
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;

    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;

    double axg = 0;
    double ayg = 0;
    double azg = 0;

    double gxrs = 0;
    double gyrs = 0;
    double gzrs = 0;

    volatile uint8_t buffer[14];

    /* Get raw data */
    while (1) {
        mpu6050_read_bit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_DATA_RDY_BIT, (uint8_t *) buffer);
        if (buffer[0])
            break;
        _delay_us(10);
    }

    mpu6050_read_bytes(MPU6050_RA_ACCEL_XOUT_H, 14, (uint8_t *) buffer);

    ax = (((int16_t) buffer[0]) << 8) | buffer[1];
    ay = (((int16_t) buffer[2]) << 8) | buffer[3];
    az = (((int16_t) buffer[4]) << 8) | buffer[5];
    gx = (((int16_t) buffer[8]) << 8) | buffer[9];
    gy = (((int16_t) buffer[10]) << 8) | buffer[11];
    gz = (((int16_t) buffer[12]) << 8) | buffer[13];

#if MPU6050_CALIBRATEDACCGYRO == 1
    axg = (double)(ax - MPU6050_AXOFFSET) / MPU6050_AXGAIN;
    ayg = (double)(ay - MPU6050_AYOFFSET) / MPU6050_AYGAIN;
    azg = (double)(az - MPU6050_AZOFFSET) / MPU6050_AZGAIN;

    gxrs = (double)(gx - MPU6050_GXOFFSET) / MPU6050_GXGAIN * 0.01745329;       /* degree to radians */
    gyrs = (double)(gy - MPU6050_GYOFFSET) / MPU6050_GYGAIN * 0.01745329;       /* degree to radians */
    gzrs = (double)(gz - MPU6050_GZOFFSET) / MPU6050_GZGAIN * 0.01745329;       /* degree to radians */
#else
    axg = (double)(ax) / MPU6050_AGAIN;
    ayg = (double)(ay) / MPU6050_AGAIN;
    azg = (double)(az) / MPU6050_AGAIN;

    gxrs = (double)(gx) / MPU6050_GGAIN * 0.01745329;   /* degree to radians */
    gyrs = (double)(gy) / MPU6050_GGAIN * 0.01745329;   /* degree to radians */
    gzrs = (double)(gz) / MPU6050_GGAIN * 0.01745329;   /* degree to radians */
#endif

    /* Compute data */
#if MPU6050_GETATTITUDE == 1
    mpu6050_mahony_update(qn, gxrs, gyrs, gzrs, axg, ayg, azg);
#endif
#if MPU6050_GETATTITUDE == 2
    mpu6050_madgwick_update(qn, gxrs, gyrs, gzrs, axg, ayg, azg);
#endif
}

#if MPU6050_GETATTITUDE == 1

#define MPU6050_MAHONY_SAMPLE_FREQ	61.0f  /* sample frequency in Hz */
#define MPU6050_MAHONY_TWO_KP_DEF	(2.0f * 0.5f)   /* 2 * proportional gain */
#define MPU6050_MAHONY_TWO_KI_DEF	(2.0f * 0.0f)   /* 2 * integral gain */

/* IMU algorithm update */
void mpu6050_mahony_update(quaternion_t *qn, float gx, float gy, float gz, float ax, float ay, float az) {

    volatile float twoKp = MPU6050_MAHONY_TWO_KP_DEF;        /* 2 * proportional gain (Kp) */
    volatile float twoKi = MPU6050_MAHONY_TWO_KI_DEF;        /* 2 * integral gain (Ki) */

    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    /* Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation) */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        /* Normalise accelerometer measurement */
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Estimated direction of gravity and vector perpendicular to magnetic flux */
        halfvx = qn->q1 * qn->q3 - qn->q0 * qn->q2;
        halfvy = qn->q0 * qn->q1 + qn->q2 * qn->q3;
        halfvz = qn->q0 * qn->q0 - 0.5f + qn->q3 * qn->q3;

        /* Error is sum of cross product between estimated and measured direction of gravity */
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        /* Compute and apply integral feedback if enabled */
        if (twoKi > 0.0f) {
            qn->integralFBx += twoKi * halfex * (1.0f / MPU6050_MAHONY_SAMPLE_FREQ); /* integral error scaled by Ki */
            qn->integralFBy += twoKi * halfey * (1.0f / MPU6050_MAHONY_SAMPLE_FREQ);
            qn->integralFBz += twoKi * halfez * (1.0f / MPU6050_MAHONY_SAMPLE_FREQ);
            gx += qn->integralFBx; /* apply integral feedback */
            gy += qn->integralFBy;
            gz += qn->integralFBz;
        } else {
            qn->integralFBx = 0.0f; /* prevent integral windup */
            qn->integralFBy = 0.0f;
            qn->integralFBz = 0.0f;
        }

        /* Apply proportional feedback */
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }
    /* Integrate rate of change of quaternion */
    gx *= (0.5f * (1.0f / MPU6050_MAHONY_SAMPLE_FREQ)); /* pre-multiply common factors */
    gy *= (0.5f * (1.0f / MPU6050_MAHONY_SAMPLE_FREQ));
    gz *= (0.5f * (1.0f / MPU6050_MAHONY_SAMPLE_FREQ));

    qa = qn->q0;
    qb = qn->q1;
    qc = qn->q2;

    qn->q0 += (-qb * gx - qc * gy - qn->q3 * gz);
    qn->q1 += (qa * gx + qc * gz - qn->q3 * gy);
    qn->q2 += (qa * gy - qb * gz + qn->q3 * gx);
    qn->q3 += (qa * gz + qb * gy - qc * gx);

    /* Normalise quaternion */
    recipNorm = inv_sqrt(qn->q0 * qn->q0 + qn->q1 * qn->q1 + qn->q2 * qn->q2 + qn->q3 * qn->q3);
    qn->q0 *= recipNorm;
    qn->q1 *= recipNorm;
    qn->q2 *= recipNorm;
    qn->q3 *= recipNorm;
}
#endif                          /* MPU6050_GETATTITUDE == 1 */

#if MPU6050_GETATTITUDE == 2

#define MPU6050_MADGWIK_SAMPLE_FREQ	61.0f   /* Sample frequency in Hz */
#define MPU6050_MADGWIK_BETA_DEF	0.1f    /* 2 * proportional gain */

/* IMU algorithm update */
void mpu6050_madgwick_update(quaternion_t *qn, float gx, float gy, float gz, float ax, float ay, float az) {

    float beta = MPU6050_MADGWIK_BETA_DEF; /* 2 * proportional gain (Kp) */

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    /*  Rate of change of quaternion from gyroscope */
    qDot1 = 0.5f * (-(qn->q1) * gx - qn->q2 * gy - qn->q3 * gz);
    qDot2 = 0.5f * (qn->q0 * gx + qn->q2 * gz - qn->q3 * gy);
    qDot3 = 0.5f * (qn->q0 * gy - qn->q1 * gz + qn->q3 * gx);
    qDot4 = 0.5f * (qn->q0 * gz + qn->q1 * gy - qn->q2 * gx);

    /*  Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation) */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        /* Normalise accelerometer measurement */
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Auxiliary variables to avoid repeated arithmetic */
        _2q0 = 2.0f * qn->q0;
        _2q1 = 2.0f * qn->q1;
        _2q2 = 2.0f * qn->q2;
        _2q3 = 2.0f * qn->q3;
        _4q0 = 4.0f * qn->q0;
        _4q1 = 4.0f * qn->q1;
        _4q2 = 4.0f * qn->q2;
        _8q1 = 8.0f * qn->q1;
        _8q2 = 8.0f * qn->q2;
        q0q0 = qn->q0 * qn->q0;
        q1q1 = qn->q1 * qn->q1;
        q2q2 = qn->q2 * qn->q2;
        q3q3 = qn->q3 * qn->q3;

        /* Gradient decent algorithm corrective step */
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * qn->q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * qn->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * qn->q3 - _2q1 * ax + 4.0f * q2q2 * qn->q3 - _2q2 * ay;

        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); /*  Normalise step magnitude */

        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        /* Apply feedback step */
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    /* Integrate rate of change of quaternion to yield quaternion */
    qn->q0 += qDot1 * (1.0f / MPU6050_MADGWIK_SAMPLE_FREQ);
    qn->q1 += qDot2 * (1.0f / MPU6050_MADGWIK_SAMPLE_FREQ);
    qn->q2 += qDot3 * (1.0f / MPU6050_MADGWIK_SAMPLE_FREQ);
    qn->q3 += qDot4 * (1.0f / MPU6050_MADGWIK_SAMPLE_FREQ);

    /* Normalise quaternion */
    recipNorm = inv_sqrt(qn->q0 * qn->q0 + qn->q1 * qn->q1 + qn->q2 * qn->q2 + qn->q3 * qn->q3);
    qn->q0 *= recipNorm;
    qn->q1 *= recipNorm;
    qn->q2 *= recipNorm;
    qn->q3 *= recipNorm;
}
#endif                          /* MPU6050_GETATTITUDE == 2 */

/* Fast inverse square-root */
float inv_sqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/*
 * Get euler angles aerospace sequence, to obtain sensor attitude:
 * 1. Rotate around sensor Z plane by yaw
 * 2. Rotate around sensor Y plane by pitch
 * 3. Rotate around sensor X plane by roll
 */

void mpu6050_get_roll_pitch_yaw(quaternion_t *qn, double *roll, double *pitch, double *yaw) {
    /* roll (x-axis rotation) */
    double sinr = 2.0 * (qn->q0 * qn->q1 + qn->q2 * qn->q3);
    double cosr = 1.0 - 2.0 * (qn->q1 * qn->q1 + qn->q2 * qn->q2);
    *roll = atan2(sinr, cosr);

    /* pitch (y-axis rotation) */
    double sinp = 2.0 * (qn->q0 * qn->q2 - qn->q3 * qn->q1);
    if (fabs(sinp) >= 1)
        *pitch = copysign(M_PI / 2, sinp);
    else
        *pitch = asin(sinp);

    /* yaw (z-axis rotation) */
    double siny = 2.0 * (qn->q0 * qn->q3 + qn->q1 * qn->q2);
    double cosy = 1.0 - 2.0 * (qn->q2 * qn->q2 + qn->q3 * qn->q3);
    *yaw = atan2(siny, cosy);
}
/* EOF */
