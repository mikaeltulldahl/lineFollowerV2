
#include "master_include.h"

uint32_t acc_divider;
float gyro_divider;

volatile float accelerometer_data[3]; //XYZ i g's
volatile float gyroscope_data[3]; //XYZ i degrees/s
volatile float magnetometer_data[3]; //XYZ i uTesla
volatile float mpu_temperature;

volatile int accelerometer_calib_data[3];
volatile float gyroscope_calib_data[3]={0,0,0}; // simple offset from startup value
float Magnetometer_ASA[3];

void initMPU9250(void){
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;


	/*
	 * SCLK PB13
	 * MISO PB14
	 * MOSI PB15
	 * CS	PB12
	 * Interupt --
	 *
	 * SPI2
	 *
	 */

	//GPIO config
	RCC_AHB1PeriphClockCmd(MPU_CLK_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(MPU_MISO_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(MPU_MOSI_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(MPU_CS_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = MPU_MOSI_GPIO_PIN; //MOSI
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//?
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //?
	GPIO_Init(MPU_MOSI_GPIO, &GPIO_InitStructure);//MOSI

	GPIO_InitStructure.GPIO_Pin = MPU_CLK_GPIO_PIN;
	GPIO_Init(MPU_CLK_GPIO, &GPIO_InitStructure);//CLK

	// OD för SDO?
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = MPU_MISO_GPIO_PIN;
	GPIO_Init(MPU_MISO_GPIO, &GPIO_InitStructure);//MISO

	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = MPU_CS_GPIO_PIN;
	GPIO_Init(MPU_CS_GPIO, &GPIO_InitStructure);// CS

	MPU9250_CS_DISABLE


	//	SDN_L;
	//	SDI_L;
	//	SEL_H;
	//	delay_ms(10);

	GPIO_PinAFConfig(MPU_CLK_GPIO, MPU_CLK_GPIO_PINSOURCE, MPU_AF_SPI);
	GPIO_PinAFConfig(MPU_MISO_GPIO, MPU_MISO_GPIO_PINSOURCE, MPU_AF_SPI);
	GPIO_PinAFConfig(MPU_MOSI_GPIO, MPU_MOSI_GPIO_PINSOURCE, MPU_AF_SPI);

	//SPI config
	MPU_SPI_CLK_CMD_ENABLE
	SPI_I2S_DeInit(MPU_SPI);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; //APB1 kör 42Mhz, mpu vill ha max 1mhz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	/* Initializes the SPI communication */
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(MPU_SPI, &SPI_InitStructure);
	SPI_Cmd(MPU_SPI, ENABLE); // enable SPI2

	delay_ms(150);

//		uint8_t test = mpu9250Read_Reg(0x75, 0);
	uint8_t i = 0;
	uint8_t MPU_Init_Data[17][2] = {
			{0x80, MPUREG_PWR_MGMT_1},     // Reset Device
			{0x01, MPUREG_PWR_MGMT_1},     // Clock Source
			{0x00, MPUREG_PWR_MGMT_2},     // Enable Acc & Gyro
			{BITS_DLPF_CFG_10HZ, MPUREG_CONFIG},         // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
			{0x18, MPUREG_GYRO_CONFIG},    // +-2000dps
			{0x08, MPUREG_ACCEL_CONFIG},   // +-4G
			{0x09, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
			{0x30, MPUREG_INT_PIN_CFG},    //
			//{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
			//{0x20, MPUREG_USER_CTRL},      // Enable AUX
			{0x20, MPUREG_USER_CTRL},       // I2C Master mode
			{0x0D, MPUREG_I2C_MST_CTRL}, //  I2C configuration multi-master  IIC 400KHz

			{AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  //Set the I2C slave addres of AK8963 and set for write.
			//{0x09, MPUREG_I2C_SLV4_CTRL},
			//{0x81, MPUREG_I2C_MST_DELAY_CTRL}, //Enable I2C delay

			{AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
			{0x01, MPUREG_I2C_SLV0_DO}, // Reset AK8963
			{0x81, MPUREG_I2C_SLV0_CTRL},  //Enable I2C and set 1 byte

			{AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
			{0x12, MPUREG_I2C_SLV0_DO}, // Register value to continuous measurement in 16bit
			{0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte

	};

	for(i=0; i<17; i++) {
		mpu9250Write_Reg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
		delay_ms(1);  //I2C must slow down the write speed, otherwise it won't work
	}

	delay_ms(100);

//	uint8_t whoami = mpu9250Read_Reg(MPUREG_WHOAMI, 0x00);

	mpu9250_set_acc_scale(BITS_ACCEL_FS_SEL_2G);
    mpu9250_set_gyro_scale(BITS_GYRO_FS_SEL_250DPS);

    mpu9250_read_acc_calib_data();
    mpu9250_read_magnetometer_calib_data();

	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; //APB1 kör 42Mhz, mpu vill ha max 20mhz på sensorvärde registren
	SPI_Init(MPU_SPI, &SPI_InitStructure);

    mpu9250_read_all();
    gyroscope_calib_data[0] = gyroscope_data[0];
    gyroscope_calib_data[1] = gyroscope_data[1];
    gyroscope_calib_data[2] = gyroscope_data[2];

    //AK8963_calib_Magnetometer();  //Can't load this function here , strange problem?



}

uint8_t mpu9250Write_Reg(uint8_t WriteAddr, uint8_t WriteData){
	uint8_t temp;
	MPU9250_CS_ENABLE
	//	(void)MPU_SPI->DR;// to clear the rxne flag

	while(SPI_I2S_GetFlagStatus(MPU_SPI, SPI_I2S_FLAG_TXE) == RESET); //waiting for spi being ready

	SPI_I2S_SendData(MPU_SPI, WriteAddr);
	while(SPI_I2S_GetFlagStatus(MPU_SPI, SPI_I2S_FLAG_RXNE) == RESET); // could change to txe
	temp=MPU_SPI->DR;
	SPI_I2S_SendData(MPU_SPI, WriteData);
	while(SPI_I2S_GetFlagStatus(MPU_SPI, SPI_I2S_FLAG_RXNE) == RESET);
	temp=MPU_SPI->DR;
	while(SPI_I2S_GetFlagStatus(MPU_SPI, SPI_I2S_FLAG_BSY) == SET);

	//tiny delay
	volatile int i;
	for(i=0;i<10;i++);

	MPU9250_CS_DISABLE
	for(i=0;i<100;i++);//probably unecessary
	return temp;
}

void mpu9250Read_Regs( uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes ){
	volatile uint8_t temp;
    volatile unsigned int  i = 0;
    MPU9250_CS_ENABLE
    while(SPI_I2S_GetFlagStatus(MPU_SPI, SPI_I2S_FLAG_TXE) == RESET); //waiting for spi being ready

    SPI_I2S_SendData(MPU_SPI, ReadAddr | READ_FLAG);
    while(SPI_I2S_GetFlagStatus(MPU_SPI, SPI_I2S_FLAG_RXNE) == RESET); // could change to txe
    temp=MPU_SPI->DR;//clear RXNE
    (void)temp;

    for(i=0; i<Bytes; i++){
    	while(SPI_I2S_GetFlagStatus(MPU_SPI, SPI_I2S_FLAG_TXE) == RESET);
    	SPI_I2S_SendData(MPU_SPI, 0x00);
        while(SPI_I2S_GetFlagStatus(MPU_SPI, SPI_I2S_FLAG_RXNE) == RESET); // could change to txe
    	ReadBuf[i] = MPU_SPI->DR;
    }
    while(SPI_I2S_GetFlagStatus(MPU_SPI, SPI_I2S_FLAG_BSY) == SET);
	//tiny delay

	for(i=0;i<1;i++);
    MPU9250_CS_DISABLE
	for(i=0;i<100;i++);//probably unecessary
}
/*-----------------------------------------------------------------------------------------------
                                ACCELEROMETER SCALE
usage: call this function at startup, after initialization, to set the right range for the
accelerometers. Suitable ranges are:
BITS_FS_2G
BITS_FS_4G
BITS_FS_8G
BITS_FS_16G
returns the range set (2,4,8 or 16)
-----------------------------------------------------------------------------------------------*/
void mpu9250_set_acc_scale(int scale){
    mpu9250Write_Reg(MPUREG_ACCEL_CONFIG, scale); // TODO read register first and mask with original content of register

    switch (scale){
        case BITS_ACCEL_FS_SEL_2G:
            acc_divider=16384;
        break;
        case BITS_ACCEL_FS_SEL_4G:
            acc_divider=8192;
        break;
        case BITS_ACCEL_FS_SEL_8G:
            acc_divider=4096;
        break;
        case BITS_ACCEL_FS_SEL_16G:
            acc_divider=2048;
        break;
    }
}

/*-----------------------------------------------------------------------------------------------
                                GYROSCOPE SCALE
usage: call this function at startup, after initialization, to set the right range for the
gyroscopes. Suitable ranges are:
BITS_FS_250DPS
BITS_FS_500DPS
BITS_FS_1000DPS
BITS_FS_2000DPS
returns the range set (250,500,1000 or 2000)
-----------------------------------------------------------------------------------------------*/
void mpu9250_set_gyro_scale(int scale){
    mpu9250Write_Reg(MPUREG_GYRO_CONFIG, scale);// TODO read register first and mask with original content of register
    switch (scale){
        case BITS_GYRO_FS_SEL_250DPS:
            gyro_divider=131;
        break;
        case BITS_GYRO_FS_SEL_500DPS:
            gyro_divider=65.5;
        break;
        case BITS_GYRO_FS_SEL_1000DPS:
            gyro_divider=32.8;
        break;
        case BITS_GYRO_FS_SEL_2000DPS:
            gyro_divider=16.4;
        break;
    }
}

/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
-----------------------------------------------------------------------------------------------*/
void mpu9250_read_accelerometer(){
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    mpu9250Read_Regs(MPUREG_ACCEL_XOUT_H,response,6);
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        accelerometer_data[i]=data/acc_divider;
    }

}

/*-----------------------------------------------------------------------------------------------
                                READ GYROSCOPE
usage: call this function to read gyroscope data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
-----------------------------------------------------------------------------------------------*/
void mpu9250_read_gyroscope(){
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    mpu9250Read_Regs(MPUREG_GYRO_XOUT_H,response,6);
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        gyroscope_data[i]=data/gyro_divider -gyroscope_calib_data[i];
    }

}

/*-----------------------------------------------------------------------------------------------
                                READ TEMPERATURE
usage: call this function to read temperature data.
returns the value in °C
-----------------------------------------------------------------------------------------------*/
void mpu9250_read_temp(){
    uint8_t response[2];
    int16_t bit_data;
    float data;
    mpu9250Read_Regs(MPUREG_TEMP_OUT_H,response,2);
    bit_data=((int16_t)response[0]<<8)|response[1];
    data=(float)bit_data;
    mpu_temperature=(data/340)+36.53;
}

/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER CALIBRATION
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns Factory Trim value
-----------------------------------------------------------------------------------------------*/
void mpu9250_read_acc_calib_data(){
    uint8_t response[4];
    int temp_scale;
    //READ CURRENT ACC SCALE
    temp_scale=mpu9250Read_Reg(MPUREG_ACCEL_CONFIG, 0x00);
    mpu9250_set_acc_scale(BITS_ACCEL_FS_SEL_8G);//calibration must be done with this scale
    //ENABLE SELF TEST need modify
    //temp_scale=WriteReg(MPUREG_ACCEL_CONFIG, 0x80>>axis);

    mpu9250Read_Regs(MPUREG_SELF_TEST_X,response,4);
    accelerometer_calib_data[0]=((response[0]&11100000)>>3)|((response[3]&00110000)>>4);
    accelerometer_calib_data[1]=((response[1]&11100000)>>3)|((response[3]&00001100)>>2);
    accelerometer_calib_data[2]=((response[2]&11100000)>>3)|((response[3]&00000011));

    mpu9250_set_acc_scale(temp_scale); //change back the scale
}
uint8_t mpu9250_AK8963_whoami(){
    uint8_t response;
    mpu9250Read_Reg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR); //Set the I2C slave addres of AK8963 and set for read.
    mpu9250Write_Reg(MPUREG_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
    mpu9250Write_Reg(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer

    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
    delay_ms(1);
    response=mpu9250Read_Reg(MPUREG_EXT_SENS_DATA_00, 0x00);    //Read I2C
    //ReadRegs(MPUREG_EXT_SENS_DATA_00,response,1);
    //response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C

    return response;
}
void mpu9250_read_magnetometer_calib_data(){
    uint8_t response[3];
    float data;
    int i;

    mpu9250Write_Reg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    mpu9250Write_Reg(MPUREG_I2C_SLV0_REG, AK8963_ASAX); //I2C slave 0 register address from where to begin data transfer
    mpu9250Write_Reg(MPUREG_I2C_SLV0_CTRL, 0x83); //Read 3 bytes from the magnetometer

    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
    delay_ms(1);
    //response[0]=WriteReg(MPUREG_EXT_SENS_DATA_01|READ_FLAG, 0x00);    //Read I2C
    mpu9250Read_Regs(MPUREG_EXT_SENS_DATA_00,response,3);

    //response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C
    for(i=0; i<3; i++) {
        data=response[i];
        Magnetometer_ASA[i]=((float)(data-128)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
    }
}
void mpu9250_read_magnetometer(){
    uint8_t response[7];
    int16_t bit_data;
    float data;
    int i;

    mpu9250Write_Reg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    mpu9250Write_Reg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    mpu9250Write_Reg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 6 bytes from the magnetometer

    delay_ms(1);
    mpu9250Read_Regs(MPUREG_EXT_SENS_DATA_00,response,7);
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2+1]<<8)|response[i*2];
        data=(float)bit_data;
        magnetometer_data[i]=data*Magnetometer_ASA[i];
    }
}
void mpu9250_read_all(){
    uint8_t response[21];
    int16_t bit_data;
    float data;
    int i;
// uncomment if you will use magnetometer
//    //Send I2C command at first
//    mpu9250Write_Reg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
//    mpu9250Write_Reg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
//    mpu9250Write_Reg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 7 bytes from the magnetometer
//    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

    //delay_ms(1);
    mpu9250Read_Regs(MPUREG_ACCEL_XOUT_H,response,21);
    //Get accelerometer value
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        accelerometer_data[i]=data/acc_divider;
    }
    //Get temperature
    bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
    data=(float)bit_data;
    mpu_temperature=((data-21)/333.87)+21;
    //Get gyroscop value
    for(i=4; i<7; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        gyroscope_data[i-4]=data/gyro_divider-gyroscope_calib_data[i-4];
    }
    //Get Magnetometer value
    for(i=7; i<10; i++) {
        bit_data=((int16_t)response[i*2+1]<<8)|response[i*2];
        data=(float)bit_data;
        magnetometer_data[i-7]=data*Magnetometer_ASA[i-7];
    }
}

