#include "IMU.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

#define MPU6050_ADDR         (0x68 << 1) //(0xD0) according to data sheet
#define PWR_MGMT_1_REG       0x6B
#define ACCEL_XOUT_H_REG     0x3B

#define MAX7219_REG_DIGIT0      0x01
#define MAX7219_REG_DECODEMODE  0x09
#define MAX7219_REG_INTENSITY   0x0A
#define MAX7219_REG_SCANLIMIT   0x0B
#define MAX7219_REG_SHUTDOWN    0x0C
#define MAX7219_REG_DISPLAYTEST 0x0F

#ifndef MAX7219_CS_GPIO_Port
#define MAX7219_CS_GPIO_Port    GPIOA
#endif

#ifndef MAX7219_CS_Pin
#define MAX7219_CS_Pin          GPIO_PIN_6
#endif
#define RAD_TO_DEG 57.29577951308232f
#define ALPHA 0.96f
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;

MPU6050_t imu_data = {0};
IMU_ZeroState imu_zero = {0};

 //  DISPLAY PATTERNS arrows (MAX7219 Matrix)
const uint8_t SMILEY_FACE[8] = {0x3C, 0x42, 0xA5, 0x81, 0xA5, 0x99, 0x42, 0x3C}; // FIRST state
const uint8_t ARROW_N[8]  = {0x18, 0x3C, 0x7E, 0x18, 0x18, 0x18, 0x18, 0x00};
const uint8_t ARROW_S[8]  = {0x18, 0x18, 0x18, 0x18, 0x7E, 0x3C, 0x18, 0x00};
const uint8_t ARROW_E[8]  = {0x10, 0x20, 0x40, 0xFF, 0x40, 0x20, 0x10, 0x00};
const uint8_t ARROW_W[8]  = {0x08, 0x04, 0x02, 0xFF, 0x02, 0x04, 0x08, 0x00};
const uint8_t ARROW_NE[8] = {0x0F, 0x03, 0x05, 0x09, 0x10, 0x20, 0x40, 0x80};
const uint8_t ARROW_NW[8] = {0xF0, 0xC0, 0xA0, 0x90, 0x08, 0x04, 0x02, 0x01};
const uint8_t ARROW_SE[8] = {0x80, 0x40, 0x20, 0x10, 0x09, 0x05, 0x03, 0x0F};
const uint8_t ARROW_SW[8] = {0x01, 0x02, 0x04, 0x08, 0x90, 0xA0, 0xC0, 0xF0};

void MAX7219_Send(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = { reg, data };
    HAL_GPIO_WritePin(MAX7219_CS_GPIO_Port, MAX7219_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, tx, 2, 10); // transmit data for matrix
    HAL_GPIO_WritePin(MAX7219_CS_GPIO_Port, MAX7219_CS_Pin, GPIO_PIN_SET);// Activate LEDs

    for (volatile int i = 0; i < 100; i++); // small brake for the register
}
void MAX7219_Init(void)
{
    MAX7219_Send(MAX7219_REG_SHUTDOWN, 0x00);    // set to zero
    HAL_Delay(10);
    MAX7219_Send(MAX7219_REG_DISPLAYTEST, 0x00); // check if matrix is of
    MAX7219_Send(MAX7219_REG_DECODEMODE, 0x00);  // matrix state
    MAX7219_Send(MAX7219_REG_SCANLIMIT, 0x07);   // Search for 8 rows
    MAX7219_Send(MAX7219_REG_INTENSITY, 0x0F);   // configure high light power
    MAX7219_Send(MAX7219_REG_SHUTDOWN, 0x01);    // go out from SHUTDOWN state
    HAL_Delay(10);
}

void MAX7219_DrawPattern(const uint8_t pattern[8])
{
    for (uint8_t row = 0; row < 8; row++) {// loop for printing pattern
        MAX7219_Send(MAX7219_REG_DIGIT0 + row, pattern[row]);
    }
}

void MPU6050_Init(void) // function to get data from IMU device
{
    uint8_t reg_addr;
    uint8_t check = 0;
    uint8_t tx_buf[2];

    reg_addr = 0x75;
    if (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg_addr, 1, 1000) == HAL_OK)// if condition to see if IMU can get data
    {
        HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, &check, 1, 1000); // Receive data for check

        if (check == 0x68)// if condition to see if check got data
        {
            tx_buf[0] = PWR_MGMT_1_REG;// get of from sleep mode
            tx_buf[1] = 0x00;// set zero to scan again
            HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, tx_buf, 2, 1000);

            tx_buf[0] = 0x1C;// configure Accelerometer
            tx_buf[1] = 0x00;// set zero to scan again
            HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, tx_buf, 2, 1000);

            tx_buf[0] = 0x1B;// configure Gyroscope (register adders of the device)
            tx_buf[1] = 0x00; // set zero to scan again
            HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, tx_buf, 2, 1000);
        }
    }
    else
    {
        printf("I2C Transmit Error! Check bus connections or pull-ups.\r\n");
    }
}

void MPU6050_Read_All(void)
{
    uint8_t Rec_Data[14];
    uint8_t reg_addr = ACCEL_XOUT_H_REG;
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg_addr, 1, 20);

    if (status == HAL_OK)
    {
        status = HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, Rec_Data, 14, 20); // receive data to Rec_Data
    }

    if (status == HAL_OK)
    {
        // Accel
        int16_t Accel_X_RAW = (int16_t)((Rec_Data[0] << 8) | Rec_Data[1]);
        int16_t Accel_Y_RAW = (int16_t)((Rec_Data[2] << 8) | Rec_Data[3]);
        int16_t Accel_Z_RAW = (int16_t)((Rec_Data[4] << 8) | Rec_Data[5]);

        // Gyro
        int16_t Gyro_X_RAW  = (int16_t)((Rec_Data[8] << 8) | Rec_Data[9]);
        int16_t Gyro_Y_RAW  = (int16_t)((Rec_Data[10] << 8) | Rec_Data[11]);
        int16_t Gyro_Z_RAW  = (int16_t)((Rec_Data[12] << 8) | Rec_Data[13]);

        imu_data.accel_x = Accel_X_RAW / 16384.0f;
        imu_data.accel_y = Accel_Y_RAW / 16384.0f;
        imu_data.accel_z = Accel_Z_RAW / 16384.0f;

        // convert to (deg/s)
        imu_data.gyro_x  = Gyro_X_RAW / 131.0f;
        imu_data.gyro_y  = Gyro_Y_RAW / 131.0f;
        imu_data.gyro_z  = Gyro_Z_RAW / 131.0f;
    }
    else
    {
    	// Handle error
        __HAL_I2C_DISABLE(&hi2c1);
        HAL_Delay(1);
        __HAL_I2C_ENABLE(&hi2c1);
    }
}

void IMU_CalibrateZero(void) // function to get first state
{
	    float local_sum_x = 0.0f;
	    float local_sum_y = 0.0f;
	    float local_sum_z = 0.0f;

	    for (int i = 0; i < 50; i++) {// get sum of accel
	        MPU6050_Read_All();
	        local_sum_x += imu_data.accel_x;
	        local_sum_y += imu_data.accel_y;
	        local_sum_z += imu_data.accel_z;
	        HAL_Delay(10);
	    }

	    imu_zero.ax_offset = local_sum_x / 50.0f;// get average of accel
	    imu_zero.ay_offset = local_sum_y / 50.0f;
	    imu_zero.az_offset = local_sum_z / 50.0f;

}
static FilteredOrientation current_orientation = {0.0f, 0.0f};

void Update_Matrix_Direction(void)
{
    const float dt = 0.1f; // sampling rate
    static int last_drawn_pattern = -1;
    float ax = imu_data.accel_x - imu_zero.ax_offset; // accel delta
    float ay = imu_data.accel_y - imu_zero.ay_offset;
    float az = imu_data.accel_z - imu_zero.az_offset;

    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG; //accel  pitch degree
    float accel_roll  = atan2f(ay, az) * RAD_TO_DEG; // convert to degrees

    current_orientation.pitch = ALPHA * (current_orientation.pitch + imu_data.gyro_y * dt)
                                + (1.0f - ALPHA) * accel_pitch;

    current_orientation.roll  = ALPHA * (current_orientation.roll + imu_data.gyro_x * dt)
                                + (1.0f - ALPHA) * accel_roll;

    float total_tilt = sqrtf(current_orientation.pitch * current_orientation.pitch + // calculate tilt degree
                             current_orientation.roll * current_orientation.roll);

    int current_pattern = 0; // default

    if (total_tilt < 18.0f) {
        current_pattern = 0; // Smile
    } else {
        float dir_angle = atan2f(current_orientation.roll, current_orientation.pitch) * RAD_TO_DEG;

        if (dir_angle >= -22.5f && dir_angle < 22.5f)        current_pattern = 1; // E
        else if (dir_angle >= 22.5f && dir_angle < 67.5f)    current_pattern = 2; // NE
        else if (dir_angle >= 67.5f && dir_angle < 112.5f)   current_pattern = 3; // N
        else if (dir_angle >= 112.5f && dir_angle < 157.5f)  current_pattern = 4; // NW
        else if (dir_angle >= 157.5f || dir_angle < -157.5f) current_pattern = 5; // W
        else if (dir_angle >= -157.5f && dir_angle < -112.5f) current_pattern = 6; // SW
        else if (dir_angle >= -112.5f && dir_angle < -67.5f)  current_pattern = 7; // S
        else if (dir_angle >= -67.5f && dir_angle < -22.5f)   current_pattern = 8; // SE
        else                                                 current_pattern = 0; //smile
    }

    // repaint if there is a change
    if (current_pattern != last_drawn_pattern)
    {
        last_drawn_pattern = current_pattern;

        switch (current_pattern) {
            case 0: MAX7219_DrawPattern(SMILEY_FACE); break;
            case 1: MAX7219_DrawPattern(ARROW_E); break;
            case 2: MAX7219_DrawPattern(ARROW_NE); break;
            case 3: MAX7219_DrawPattern(ARROW_N); break;
            case 4: MAX7219_DrawPattern(ARROW_NW); break;
            case 5: MAX7219_DrawPattern(ARROW_W); break;
            case 6: MAX7219_DrawPattern(ARROW_SW); break;
            case 7: MAX7219_DrawPattern(ARROW_S); break;
            case 8: MAX7219_DrawPattern(ARROW_SE); break;
        }
    }
}
