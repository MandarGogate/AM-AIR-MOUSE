/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#include "tm_stm32f4_mpu6050.h"

#define MPU6050_RA_XA_OFFS_H 0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC 0x07
#define MPU6050_RA_YA_OFFS_H 0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC 0x09
#define MPU6050_RA_ZA_OFFS_H 0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC 0x0B
/*** ACCELEROMETER OFFSETS ***/

int16_t _mpu6050_getXAccOffset(TM_MPU6050_t* self) {
	uint8_t buf[2];
//uint8_t TM_I2C_Read(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg);
	//TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_CONFIG);
	buf[0] = TM_I2C_Read(MPU6050_I2C, self->Address, MPU6050_RA_XA_OFFS_H);
	buf[1] = TM_I2C_Read(MPU6050_I2C, self->Address, MPU6050_RA_XA_OFFS_L_TC);
  return (((int16_t)buf[0]) << 8) | buf[1];
}

void _mpu6050_setXAccOffset(TM_MPU6050_t* self, int16_t offset) {
	uint8_t buf[2] = {offset >> 8, offset};
	//void TM_I2C_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t data);
	//mpu6050_writeByte(self, MPU6050_RA_XA_OFFS_H, buf[0]);
	//mpu6050_writeByte(self, MPU6050_RA_XA_OFFS_L_TC, buf[1]);
	TM_I2C_Write(MPU6050_I2C, self->Address, MPU6050_RA_XA_OFFS_H, buf[0]);
	TM_I2C_Write(MPU6050_I2C, self->Address, MPU6050_RA_XA_OFFS_L_TC,buf[1]);

}
int16_t _mpu6050_getYAccOffset(TM_MPU6050_t* self) {
	uint8_t buf[2];
//uint8_t TM_I2C_Read(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg);
	//TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_CONFIG);
	buf[0] = TM_I2C_Read(MPU6050_I2C, self->Address, MPU6050_RA_YA_OFFS_H);
	buf[1] = TM_I2C_Read(MPU6050_I2C, self->Address, MPU6050_RA_YA_OFFS_L_TC);
  return (((int16_t)buf[0]) << 8) | buf[1];
}

void _mpu6050_setYAccOffset(TM_MPU6050_t* self, int16_t offset) {
	uint8_t buf[2] = {offset >> 8, offset};
	//void TM_I2C_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t data);
	//mpu6050_writeByte(self, MPU6050_RA_XA_OFFS_H, buf[0]);
	//mpu6050_writeByte(self, MPU6050_RA_XA_OFFS_L_TC, buf[1]);
	TM_I2C_Write(MPU6050_I2C, self->Address, MPU6050_RA_YA_OFFS_H, buf[0]);
	TM_I2C_Write(MPU6050_I2C, self->Address, MPU6050_RA_YA_OFFS_L_TC,buf[1]);

}
int16_t _mpu6050_getZAccOffset(TM_MPU6050_t* self) {
	uint8_t buf[2];
//uint8_t TM_I2C_Read(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg);
	//TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_CONFIG);
	buf[0] = TM_I2C_Read(MPU6050_I2C, self->Address, MPU6050_RA_ZA_OFFS_H);
	buf[1] = TM_I2C_Read(MPU6050_I2C, self->Address, MPU6050_RA_ZA_OFFS_L_TC);
  return (((int16_t)buf[0]) << 8) | buf[1];
}

void _mpu6050_setZAccOffset(TM_MPU6050_t* self, int16_t offset) {
	uint8_t buf[2] = {offset >> 8, offset};
	//void TM_I2C_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t data);
	//mpu6050_writeByte(self, MPU6050_RA_XA_OFFS_H, buf[0]);
	//mpu6050_writeByte(self, MPU6050_RA_XA_OFFS_L_TC, buf[1]);
	TM_I2C_Write(MPU6050_I2C, self->Address, MPU6050_RA_ZA_OFFS_H, buf[0]);
	TM_I2C_Write(MPU6050_I2C, self->Address, MPU6050_RA_ZA_OFFS_L_TC,buf[1]);

}
TM_MPU6050_Result_t TM_MPU6050_Init(TM_MPU6050_t* DataStruct, TM_MPU6050_Device_t DeviceNumber, TM_MPU6050_Accelerometer_t AccelerometerSensitivity, TM_MPU6050_Gyroscope_t GyroscopeSensitivity) {
	uint8_t temp;
	int16_t offX,offY,offZ;
	/* Format I2C address */
	DataStruct->Address = MPU6050_I2C_ADDR | (uint8_t)DeviceNumber;
	
	/* Initialize I2C */
	TM_I2C_Init(MPU6050_I2C, MPU6050_I2C_PINSPACK, MPU6050_I2C_CLOCK);
	
	/* Check if device is connected */
	if (!TM_I2C_IsDeviceConnected(MPU6050_I2C, DataStruct->Address)) {
		/* Return error */
		return TM_MPU6050_Result_DeviceNotConnected;
	}
	
	/* Check who I am */
	if (TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_WHO_AM_I) != MPU6050_I_AM) {
		/* Return error */
		return TM_MPU6050_Result_DeviceInvalid;
	}
	
	/* Wakeup MPU6050 */
	TM_I2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_PWR_MGMT_1, 0x00);
	
	/* Config accelerometer */
	temp = TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	TM_I2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_CONFIG, temp);
	
	/* Config gyroscope */
	temp = TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_GYRO_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	TM_I2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_GYRO_CONFIG, temp);
	
	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case TM_MPU6050_Accelerometer_2G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_2; 
			break;
		case TM_MPU6050_Accelerometer_4G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_4; 
			break;
		case TM_MPU6050_Accelerometer_8G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_8; 
			break;
		case TM_MPU6050_Accelerometer_16G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_16; 
		default:
			break;
	}
	
	switch (GyroscopeSensitivity) {
		case TM_MPU6050_Gyroscope_250s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250; 
			break;
		case TM_MPU6050_Gyroscope_500s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500; 
			break;
		case TM_MPU6050_Gyroscope_1000s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000; 
			break;
		case TM_MPU6050_Gyroscope_2000s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000; 
		default:
			break;
	}

//	offX = _mpu6050_getXAccOffset(DataStruct);
//	_mpu6050_setXAccOffset(DataStruct,offX);
//	offY = _mpu6050_getYAccOffset(DataStruct);
//	_mpu6050_setYAccOffset(DataStruct,offY);
//	offZ = _mpu6050_getZAccOffset(DataStruct);
//	_mpu6050_setZAccOffset(DataStruct,offZ);

	/* Return OK */
	return TM_MPU6050_Result_Ok;
}

TM_MPU6050_Result_t TM_MPU6050_ReadAccelerometer(TM_MPU6050_t* DataStruct) {
	uint8_t data[6];
	/* Read accelerometer data */
	TM_I2C_ReadMulti(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_XOUT_H, data, 6);
	
	/* Format */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);	
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return TM_MPU6050_Result_Ok;
}

//int16_t _mpu6050_getYAccOffset(TM_MPU6050_t* self) {
//	uint8_t buf[2];
//	mpu6050_readByte(self, MPU6050_RA_YA_OFFS_H, &buf[0]);
//	mpu6050_readByte(self, MPU6050_RA_YA_OFFS_L_TC, &buf[1]);
//  return (((int16_t)buf[0]) << 8) | buf[1];
//}
//
//void _mpu6050_setYAccOffset(TM_MPU6050_t* self, int16_t offset) {
//	uint8_t buf[2] = {offset >> 8, offset};
//	mpu6050_writeByte(self, MPU6050_RA_YA_OFFS_H, buf[0]);
//	mpu6050_writeByte(self, MPU6050_RA_YA_OFFS_L_TC, buf[1]);
//}
//
//int16_t _mpu6050_getZAccOffset(TM_MPU6050_t* self) {
//	uint8_t buf[2];
//	mpu6050_readByte(self, MPU6050_RA_ZA_OFFS_H, &buf[0]);
//	mpu6050_readByte(self, MPU6050_RA_ZA_OFFS_L_TC, &buf[1]);
//  return (((int16_t)buf[0]) << 8) | buf[1];
//}
//
//void _mpu6050_setZAccOffset(TM_MPU6050_t* self, int16_t offset) {
//	uint8_t buf[2] = {offset >> 8, offset};
//	mpu6050_writeByte(self, MPU6050_RA_ZA_OFFS_H, buf[0]);
//	mpu6050_writeByte(self, MPU6050_RA_ZA_OFFS_L_TC, buf[1]);
//}


TM_MPU6050_Result_t TM_MPU6050_ReadGyroscope(TM_MPU6050_t* DataStruct) {
	uint8_t data[6];
	
	/* Read gyroscope data */
	TM_I2C_ReadMulti(MPU6050_I2C, DataStruct->Address, MPU6050_GYRO_XOUT_H, data, 6);
	
	/* Format */
	DataStruct->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return TM_MPU6050_Result_Ok;
}

TM_MPU6050_Result_t TM_MPU6050_ReadTemperature(TM_MPU6050_t* DataStruct) {
	uint8_t data[2];
	int16_t temp;
	
	/* Read temperature */
	TM_I2C_ReadMulti(MPU6050_I2C, DataStruct->Address, MPU6050_TEMP_OUT_H, data, 2);
	
	/* Format temperature */
	temp = (data[0] << 8 | data[1]);
	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
	
	/* Return OK */
	return TM_MPU6050_Result_Ok;
}

TM_MPU6050_Result_t TM_MPU6050_ReadAll(TM_MPU6050_t* DataStruct) {
	uint8_t data[14];
	int16_t temp;
	
	/* Read full raw data, 14bytes */
	TM_I2C_ReadMulti(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_XOUT_H, data, 14);
	
	/* Format accelerometer data */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);	
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);
	
	/* Format gyroscope data */
	DataStruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	DataStruct->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);

	/* Return OK */
	return TM_MPU6050_Result_Ok;
}
