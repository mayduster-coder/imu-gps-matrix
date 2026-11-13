#ifndef IMU_H
#define IMU_H

#include "stm32f7xx_hal.h"
#include <math.h>
#define TILT_THRESHOLD_DEG 15.0f

typedef struct {
    float ax_offset;
    float ay_offset;
    float az_offset;
} IMU_ZeroState;
typedef struct {
    float pitch;
    float roll;
} FilteredOrientation;
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} MPU6050_t;
void MATRIX_Init(void);
void MATRIX_DisplayPattern(const uint8_t pattern[8]);
void MATRIX_Clear(void);
void MAX7219_Init(void);
void MAX7219_DrawPattern(const uint8_t pattern[8]);
void MAX7219_Send(uint8_t reg, uint8_t data);

extern IMU_ZeroState imu_zero;
extern MPU6050_t imu_data;

void IMU_CalibrateZero(void);
void Update_Matrix_Direction(void);

void MPU6050_Init(void);
void MPU6050_Read_All(void);
#endif
