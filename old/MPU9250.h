#ifndef MPU9250_H
#define MPU9250_H

#include "master_include.h"

extern volatile float accelerometer_data[3]; //XYZ i g's
extern volatile float gyroscope_data[3]; //XYZ i degrees/s
extern volatile float magnetometer_data[3]; //XYZ i uTesla
extern volatile float mpu_temperature;

//#define MPU_ON_TFT
//#define MPU_ONBOARD
#define MPU_F4breakout

#ifdef MPU_ON_TFT
/*
 * mpu on tft-port
 * SCLK PPB13
 * MISO PB14
 * MOSI PB15
 * CS	PB12
 * Interupt -
 *
 * SPI2
 */
#define MPU_CLK_GPIO GPIOB
#define MPU_CLK_GPIO_CLK RCC_AHB1Periph_GPIOB
#define MPU_CLK_GPIO_PIN GPIO_Pin_13
#define MPU_CLK_GPIO_PINSOURCE GPIO_PinSource13

#define MPU_MISO_GPIO GPIOB
#define MPU_MISO_GPIO_CLK RCC_AHB1Periph_GPIOB
#define MPU_MISO_GPIO_PIN GPIO_Pin_14
#define MPU_MISO_GPIO_PINSOURCE GPIO_PinSource14

#define MPU_MOSI_GPIO GPIOB
#define MPU_MOSI_GPIO_CLK RCC_AHB1Periph_GPIOB
#define MPU_MOSI_GPIO_PIN GPIO_Pin_15
#define MPU_MOSI_GPIO_PINSOURCE GPIO_PinSource15

#define MPU_CS_GPIO GPIOB
#define MPU_CS_GPIO_CLK RCC_AHB1Periph_GPIOB
#define MPU_CS_GPIO_PIN GPIO_Pin_12

#define MPU_SPI SPI2
#define MPU_SPI_CLK_CMD_ENABLE RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
#define MPU_AF_SPI GPIO_AF_SPI2

#elif defined MPU_ONBOARD
	/*
	 * onboard mpu
	 * SCLK PA5
	 * MISO PA6
	 * MOSI PA7
	 * CS	PC4
	 * Interupt PA4
	 *
	 * SPI1
	 *
	 */
#define MPU_CLK_GPIO GPIOA
#define MPU_CLK_GPIO_CLK RCC_AHB1Periph_GPIOA
#define MPU_CLK_GPIO_PIN GPIO_Pin_5
#define MPU_CLK_GPIO_PINSOURCE GPIO_PinSource5

#define MPU_MISO_GPIO GPIOA
#define MPU_MISO_GPIO_CLK RCC_AHB1Periph_GPIOA
#define MPU_MISO_GPIO_PIN GPIO_Pin_6
#define MPU_MISO_GPIO_PINSOURCE GPIO_PinSource6

#define MPU_MOSI_GPIO GPIOA
#define MPU_MOSI_GPIO_CLK RCC_AHB1Periph_GPIOA
#define MPU_MOSI_GPIO_PIN GPIO_Pin_7
#define MPU_MOSI_GPIO_PINSOURCE GPIO_PinSource7

#define MPU_CS_GPIO GPIOC
#define MPU_CS_GPIO_CLK RCC_AHB1Periph_GPIOC
#define MPU_CS_GPIO_PIN GPIO_Pin_4

#define MPU_SPI SPI1
#define MPU_SPI_CLK_CMD_ENABLE RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
#define MPU_AF_SPI GPIO_AF_SPI1

#elif defined MPU_F4breakout
	/*
	 * onboard mpu
	 * SCLK PB13
	 * MISO PB14
	 * MOSI PB15
	 * CS	PB12
	 * Interupt PB11
	 *
	 * SPI2
	 *
	 */
#define MPU_CLK_GPIO GPIOB
#define MPU_CLK_GPIO_CLK RCC_AHB1Periph_GPIOB
#define MPU_CLK_GPIO_PIN GPIO_Pin_13
#define MPU_CLK_GPIO_PINSOURCE GPIO_PinSource13

#define MPU_MISO_GPIO GPIOB
#define MPU_MISO_GPIO_CLK RCC_AHB1Periph_GPIOB
#define MPU_MISO_GPIO_PIN GPIO_Pin_14
#define MPU_MISO_GPIO_PINSOURCE GPIO_PinSource14

#define MPU_MOSI_GPIO GPIOB
#define MPU_MOSI_GPIO_CLK RCC_AHB1Periph_GPIOB
#define MPU_MOSI_GPIO_PIN GPIO_Pin_15
#define MPU_MOSI_GPIO_PINSOURCE GPIO_PinSource15

#define MPU_CS_GPIO GPIOB
#define MPU_CS_GPIO_CLK RCC_AHB1Periph_GPIOB
#define MPU_CS_GPIO_PIN GPIO_Pin_12

#define MPU_SPI SPI2
#define MPU_SPI_CLK_CMD_ENABLE RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
#define MPU_AF_SPI GPIO_AF_SPI2
#endif


#define MPU9250_CS_ENABLE GPIO_ResetBits(MPU_CS_GPIO, MPU_CS_GPIO_PIN); //Chip enable
#define MPU9250_CS_DISABLE GPIO_SetBits(MPU_CS_GPIO, MPU_CS_GPIO_PIN); //Chip enable

void initMPU9250(void);
uint8_t mpu9250Write_Reg(uint8_t WriteAddr, uint8_t WriteData);
#define mpu9250Read_Reg(WriteAddr, WriteData)  mpu9250Write_Reg((WriteAddr) | READ_FLAG,WriteData)
void mpu9250_set_acc_scale(int scale);
void mpu9250_set_gyro_scale(int scale);

void mpu9250_read_accelerometer(void);
void mpu9250_read_gyroscope(void);
void mpu9250_read_temp(void);

void mpu9250_read_acc_calib_data(void);
void mpu9250_read_magnetometer_calib_data(void);

uint8_t mpu9250_AK8963_whoami(void);
void mpu9250_read_magnetometer(void);
void mpu9250_read_all(void);

#endif
