/*
MPU6050 lib 0x02

copyright (c) Davide Gironi, 2012

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
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

/*  Write 1 byte to chip register */
void mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
    return mpu6050_write_bytes(reg_addr, 1, &data);
}

/* Read bits from chip register */
int8_t mpu6050_read_bits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t * data) {
    /*  01101001 read byte */
    /*  76543210 bit numbers */
    /*     xxx   args: bit_start=4, length=3 */
    /*     010   masked */
    /*    -> 010 shifted */
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
    /*       010 value to write */
    /*  76543210 bit numbers */
    /*     xxx   args: bit_start=4, length=3 */
    /*  00011100 mask byte */
    /*  10101111 original value (sample) */
    /*  10100011 original & ~mask */
    /*  10101011 masked | value */
    if (length > 0) {
        uint8_t b = 0;
        if (mpu6050_read_byte(reg_addr, &b) != 0) {       /* get current data */
            uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
            data <<= (bit_start - length + 1);   /*  shift data into correct position */
            data &= mask;       /*  zero all non-important bits in data */
            b &= ~(mask);       /*  zero all important bits in existing byte */
            b |= data;          /*  combine data with existing byte */
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
    mpu6050_write_byte(MPU6050_RA_SMPLRT_DIV, 4);        /* 1khz / (1 + 4) = 200Hz */
    /* Set gyro range */
    mpu6050_write_bits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS);
    /* Set accel range */
    mpu6050_write_bits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS);

#if MPU6050_GETATTITUDE == 1
    //MPU6050_TIMER0INIT
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

#if MPU6050_GETATTITUDE == 1

/* Update timer for attitude */
//ISR(TIMER0_OVF_vect) {
//    mpu6050_update_quaternion();
//}

volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

/* Update quaternion */
void mpu6050_update_quaternion(void) {
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
    mpu6050_mahony_update(gxrs, gyrs, gzrs, axg, ayg, azg);
}

/* Mahony update function (for 6DOF) */
void mpu6050_mahony_update(float gx, float gy, float gz, float ax, float ay, float az) {

    uint8_t buffer[14];

    float norm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    /*  Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation) */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        /*  Normalise accelerometer measurement */
        norm = sqrt(ax * ax + ay * ay + az * az);
        ax /= norm;
        ay /= norm;
        az /= norm;

        /*  Estimated direction of gravity and vector perpendicular to magnetic flux */
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        /*  Error is sum of cross product between estimated and measured direction of gravity */
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        /*  Compute and apply integral feedback if enabled */
        if (MPU6050_MAHONY_TWO_KI_DEF > 0.0f) {
            integralFBx += MPU6050_MAHONY_TWO_KI_DEF * halfex * (1.0f / MPU6050_MAHONY_SAMPLE_FREQ); /*  integral error scaled by Ki */
            integralFBy += MPU6050_MAHONY_TWO_KI_DEF * halfey * (1.0f / MPU6050_MAHONY_SAMPLE_FREQ);
            integralFBz += MPU6050_MAHONY_TWO_KI_DEF * halfez * (1.0f / MPU6050_MAHONY_SAMPLE_FREQ);
            gx += integralFBx;  /*  Apply integral feedback */
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f; /*  Prevent integral windup */
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        /*  Apply proportional feedback */
        gx += MPU6050_MAHONY_TWO_KP_DEF * halfex;
        gy += MPU6050_MAHONY_TWO_KP_DEF * halfey;
        gz += MPU6050_MAHONY_TWO_KP_DEF * halfez;
    }
    /*  Integrate rate of change of quaternion */
    gx *= (0.5f * (1.0f / MPU6050_MAHONY_SAMPLE_FREQ));   /*  pre-multiply common factors */
    gy *= (0.5f * (1.0f / MPU6050_MAHONY_SAMPLE_FREQ));
    gz *= (0.5f * (1.0f / MPU6050_MAHONY_SAMPLE_FREQ));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    /*  Normalise quaternion */
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

/* Get quaternion */
void mpu6050_get_quaternion(double *qw, double *qx, double *qy, double *qz) {
    *qw = q0;
    *qx = q1;
    *qy = q2;
    *qz = q3;
}

/*
 * Get euler angles
 * Aerospace sequence, to obtain sensor attitude:
 * 1. rotate around sensor Z plane by yaw
 * 2. rotate around sensor Y plane by pitch
 * 3. rotate around sensor X plane by roll
 */
void mpu6050_get_roll_pitch_yaw(double *roll, double *pitch, double *yaw) {
    *yaw = atan2(2 * q1 * q2 - 2 * q0 * q3, 2 * q0 * q0 + 2 * q1 * q1 - 1);
    *pitch = -asin(2 * q1 * q3 + 2 * q0 * q2);
    *roll = atan2(2 * q2 * q3 - 2 * q0 * q1, 2 * q0 * q0 + 2 * q3 * q3 - 1);
}

#endif /* MPU6050_GETATTITUDE == 1 */


#if MPU6050_GETATTITUDE == 2

/* Write word/words to chip register */
void mpu6050_write_words(uint8_t reg_addr, uint8_t length, uint16_t * data) {
    if (length > 0) {
        uint8_t i = 0;
        /* Write data */
        i2c_start(MPU6050_ADDR | I2C_WRITE);
        i2c_write(reg_addr);
        for (i = 0; i < length * 2; i++) {
            i2c_write((uint8_t) (data[i++] >> 8));      /*  send MSB */
            i2c_write((uint8_t) data[i]);       /*  send LSB */
        }
        i2c_stop();
    }
}

/* Set a chip memory bank */
void mpu6050_set_memory_bank(uint8_t bank, uint8_t prefetch_enabled, uint8_t user_bank) {
    bank &= 0x1F;
    if (user_bank)
        bank |= 0x20;
    if (prefetch_enabled)
        bank |= 0x40;
    mpu6050_write_byte(MPU6050_RA_BANK_SEL, bank);
}

/* Set memory start address */
void mpu6050_set_memory_start_address(uint8_t address) {
    mpu6050_write_byte(MPU6050_RA_MEM_START_ADDR, address);
}

/* Read a memory block */
void mpu6050_read_memory_block(uint8_t * data, uint16_t data_size, uint8_t bank, uint8_t address) {
    mpu6050_set_memory_bank(bank, 0, 0);
    mpu6050_set_memory_start_address(address);
    uint8_t chunk_size;
    for (uint16_t i = 0; i < data_size;) {
        /*  determine correct chunk size according to bank position and data size */
        chunk_size = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        /*  make sure we don't go past the data size */
        if (i + chunk_size > data_size)
            chunk_size = data_size - i;

        /*  make sure this chunk doesn't go past the bank boundary (256 bytes) */
        if (chunk_size > 256 - address)
            chunk_size = 256 - address;

        /*  read the chunk of data as specified */
        mpu6050_read_bytes(MPU6050_RA_MEM_R_W, chunk_size, data + i);

        /*  increase byte index by [chunk_size] */
        i += chunk_size;

        /*  uint8_t automatically wraps to 0 at 256 */
        address += chunk_size;

        /*  if we aren't done, update bank (if necessary) and address */
        if (i < data_size) {
            if (address == 0)
                bank++;
            mpu6050_set_memory_bank(bank, 0, 0);
            mpu6050_set_memory_start_address(address);
        }
    }
}

/* Write a memory block */
uint8_t mpu6050_write_memory_block(const uint8_t * data, uint16_t data_size, uint8_t bank, uint8_t address, uint8_t verify,
                                 uint8_t use_prog_mem) {
    mpu6050_set_memory_bank(bank, 0, 0);
    mpu6050_set_memory_start_address(address);
    uint8_t chunk_size;
    uint8_t *verify_buffer = 0;
    uint8_t *prog_buffer = 0;
    uint16_t i;
    uint8_t j;
    if (verify)
        verify_buffer = (uint8_t *) malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    if (use_prog_mem)
        prog_buffer = (uint8_t *) malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    for (i = 0; i < data_size;) {
        /*  determine correct chunk size according to bank position and data size */
        chunk_size = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        /*  make sure we don't go past the data size */
        if (i + chunk_size > data_size)
            chunk_size = data_size - i;

        /*  make sure this chunk doesn't go past the bank boundary (256 bytes) */
        if (chunk_size > 256 - address)
            chunk_size = 256 - address;

        if (use_prog_mem) {
            /*  write the chunk of data as specified */
            for (j = 0; j < chunk_size; j++)
                prog_buffer[j] = pgm_read_byte(data + i + j);
        } else {
            /*  write the chunk of data as specified */
            prog_buffer = (uint8_t *) data + i;
        }

        mpu6050_write_bytes(MPU6050_RA_MEM_R_W, chunk_size, prog_buffer);

        /*  verify data if needed */
        if (verify && verify_buffer) {
            mpu6050_set_memory_bank(bank, 0, 0);
            mpu6050_set_memory_start_address(address);
            mpu6050_read_bytes(MPU6050_RA_MEM_R_W, chunk_size, verify_buffer);
            if (memcmp(prog_buffer, verify_buffer, chunk_size) != 0) {
                free(verify_buffer);
                if (use_prog_mem)
                    free(prog_buffer);
                return 0;       /*  uh oh. */
            }
        }
        /*  increase byte index by [chunk_size] */
        i += chunk_size;

        /*  uint8_t automatically wraps to 0 at 256 */
        address += chunk_size;

        /*  if we aren't done, update bank (if necessary) and address */
        if (i < data_size) {
            if (address == 0)
                bank++;
            mpu6050_set_memory_bank(bank, 0, 0);
            mpu6050_set_memory_start_address(address);
        }
    }
    if (verify)
        free(verify_buffer);
    if (use_prog_mem)
        free(prog_buffer);
    return 1;
}

/* Write a DMP configuration set */
uint8_t mpu6050_write_dmp_configuration_set(const uint8_t * data, uint16_t data_size, uint8_t use_prog_mem) {
    uint8_t *prog_buffer = 0;
    uint8_t success, special;
    uint16_t i, j;
    if (use_prog_mem) {
        prog_buffer = (uint8_t *) malloc(8);     /*  assume 8-byte blocks, realloc later if necessary */
    }
    /*  config set data is a long string of blocks with the following structure: */
    /*  [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]] */
    uint8_t bank, offset, length;
    for (i = 0; i < data_size;) {
        if (use_prog_mem) {
            bank = pgm_read_byte(data + i++);
            offset = pgm_read_byte(data + i++);
            length = pgm_read_byte(data + i++);
        } else {
            bank = data[i++];
            offset = data[i++];
            length = data[i++];
        }

        /*  write data or perform special action */
        if (length > 0) {
            /*  regular block of data to write */
            if (use_prog_mem) {
                if (sizeof(prog_buffer) < length)
                    prog_buffer = (uint8_t *) realloc(prog_buffer, length);
                for (j = 0; j < length; j++)
                    prog_buffer[j] = pgm_read_byte(data + i + j);
            } else {
                prog_buffer = (uint8_t *) data + i;
            }
            success = mpu6050_write_memory_block(prog_buffer, length, bank, offset, 1, 0);
            i += length;
        } else {
            /*  special instruction */
            /*  NOTE: this kind of behavior (what and when to do certain things) */
            /*  is totally undocumented. This code is in here based on observed */
            /*  behavior only, and exactly why (or even whether) it has to be here */
            /*  is anybody's guess for now. */
            if (use_prog_mem) {
                special = pgm_read_byte(data + i++);
            } else {
                special = data[i++];
            }
            if (special == 0x01) {
                /*  enable DMP-related interrupts */

                /* mpu6050_write_bit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, 1); //setIntZeroMotionEnabled */
                /* mpu6050_write_bit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, 1); //setIntFIFOBufferOverflowEnabled */
                /* mpu6050_write_bit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, 1); //setIntDMPEnabled */
                mpu6050_write_byte(MPU6050_RA_INT_ENABLE, 0x32); /*  single operation */

                success = 1;
            } else {
                /*  unknown special command */
                success = 0;
            }
        }

        if (!success) {
            if (use_prog_mem)
                free(prog_buffer);
            return 0;           /*  uh oh */
        }
    }
    if (use_prog_mem)
        free(prog_buffer);
    return 1;
}

/* Get the fifo count */
uint16_t mpu6050_get_fifo_count() {
    mpu6050_read_bytes(MPU6050_RA_FIFO_COUNTH, 2, (uint8_t *) buffer);
    return (((uint16_t) buffer[0]) << 8) | buffer[1];
}

/* Read fifo bytes */
void mpu6050_get_fifo_bytes(uint8_t * data, uint8_t length) {
    mpu6050_read_bytes(MPU6050_RA_FIFO_R_W, length, data);
}

/* Get the interrupt status */
uint8_t mpu6050_get_int_status() {
    volatile uint8_t buffer[14];
    mpu6050_read_byte(MPU6050_RA_INT_STATUS, (uint8_t *) buffer);
    return buffer[0];
}

/* Reset fifo */
void mpu6050_reset_fifo() {
    mpu6050_write_bit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1);
}

/* Get gyro offset X */
int8_t mpu6050_get_X_gyro_offset() {
    mpu6050_read_bits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, (uint8_t *) buffer);
    return buffer[0];
}

/* Set gyro offset X */
void mpu6050_set_X_gyro_offset(int8_t offset) {
    mpu6050_write_bits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

/* get gyro offset Y */
int8_t mpu6050_get_Y_gyro_offset() {
    mpu6050_read_bits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, (uint8_t *) buffer);
    return buffer[0];
}

/* Set gyro offset Y */
void mpu6050_set_Y_gyro_offset(int8_t offset) {
    mpu6050_write_bits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

/* Get gyro offset Z */
int8_t mpu6050_get_Z_gyro_offset() {
    mpu6050_read_bits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, (uint8_t *) buffer);
    return buffer[0];
}

/* Set gyro offset Z */
void mpu6050_set_Z_gyro_offset(int8_t offset) {
    mpu6050_write_bits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

#endif /* MPU6050_GETATTITUDE == 2 */
/* EOF */

