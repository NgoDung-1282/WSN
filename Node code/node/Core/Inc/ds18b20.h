#ifndef __DS18B20_H
#define __DS18B20_H

#include "main.h"
// #include "stm32f1xx_hal_tim.h"
#include "delay.h"

#define DS18B20_SKIP_ROM 0xCC		  // gui tin hieu den tat ca Slave
#define DS18B20_CONVERT_T 0x44		  // bat dau chuyen doi nhiet do
#define DS18B20_READ_SCRATCH_PAD 0xBE // Doc du lieu tu Slave

unsigned char ds18b20_reset();
void ds18b20_write_bit(unsigned char bit);	 // ham gui 1 bit vao DS18B20
unsigned char ds18b20_read_bit();			 // ham doc 1 bit nhan ve tu DS18B20
void ds18b20_write_byte(unsigned char byte); // ham gui 1 byte vao DS18B20
unsigned char ds18b20_read_byte();			 // ham doc 1 byte nhan ve tu DS18B20
float temperature();

TIM_HandleTypeDef htim4;

// set pin output
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// set pin input
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// Return 0: OK ; 1:FAIL
unsigned char ds18b20_reset()
{
	unsigned char result;
	Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);
	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, 0);

	delay_us(&htim4, 480);
	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, 1);
	Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);
	delay_us(&htim4, 70);

	result = HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin);
	delay_us(&htim4, 410);

	return result;
}
// ham gui 1 bit vao DS18B20
void ds18b20_write_bit(unsigned char bit)
{
	if (bit == 1)
	{
		Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);
		HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, 0);

		delay_us(&htim4, 6);

		HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, 1);
		delay_us(&htim4, 64);
	}
	else
	{

		Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);
		HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, 0);

		delay_us(&htim4, 60);

		HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, 1);

		delay_us(&htim4, 10);
	}
}
// ham doc 1 bit nhan ve tu DS18B20
unsigned char ds18b20_read_bit()
{
	unsigned char result = 0;

	Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);
	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, 0);

	delay_us(&htim4, 6);

	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, 1);
	Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);
	delay_us(&htim4, 9);

	result = HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin);

	delay_us(&htim4, 55);
	return result;
}
// ham ghi 1 byte vao DS18B20 (gui bit thap nhat truoc)
void ds18b20_write_byte(unsigned char byte)
{
	unsigned char i = 8;
	while (i--)
	{
		ds18b20_write_bit(byte & 0x01); // gui bit co trong so thap nhat (b=1010 1111 & 0000 0001 -> 0000 0001)
		byte >>= 1;						// byte >>=1 -> 0101 0111
	}
}
// ham doc 1 byte nhan ve tu DS18B20
unsigned char ds18b20_read_byte()
{
	unsigned char i = 8, result = 0;
	while (i--)
	{
		result >>= 1;
		result |= ds18b20_read_bit() << 7; // 0000 0001 << 7 = 1000 0000
	}
	return result;
}

// float temperature(){
//	uint8_t tempL, tempH;
//	float temp = 0;
//	//Ket noi den Slave, bat dau qua trinh chuyen doi nhiet do
//	while(ds18b20_reset());
//	ds18b20_write_byte(DS18B20_SKIP_ROM);
//	ds18b20_write_byte(DS18B20_CONVERT_T);
//	HAL_Delay(94);   //che do 9 bit mat 93.75ms de chuyen doi

//	//Ket noi den Slave, bat dau doc du lieu nhiet do
//	while(ds18b20_reset());
//	ds18b20_write_byte(DS18B20_SKIP_ROM);
//	ds18b20_write_byte(DS18B20_READ_SCRATCH_PAD);

//	tempL = ds18b20_read_byte();
//	tempH = ds18b20_read_byte();

//	temp = ((tempH << 8)+ tempL)*1.0f/16; //lay nhiet do tu temp_Register
//	return temp;
//}

float temperature()
{
	//	uint8_t tempL, tempH;
	unsigned int temp;
	//	float temp = 0;
	// Ket noi den Slave, bat dau qua trinh chuyen doi nhiet do
	while (ds18b20_reset())
		;
	ds18b20_write_byte(DS18B20_SKIP_ROM);
	ds18b20_write_byte(DS18B20_CONVERT_T);
	HAL_Delay(94); // che do 9 bit mat 93.75ms de chuyen doi

	// Ket noi den Slave, bat dau doc du lieu nhiet do
	while (ds18b20_reset())
		;
	ds18b20_write_byte(DS18B20_SKIP_ROM);
	ds18b20_write_byte(DS18B20_READ_SCRATCH_PAD);

	temp = ds18b20_read_byte();
	temp = (ds18b20_read_byte() << 8) | temp;

	//	temp = ((tempH << 8)+ tempL)*1.0f/16; //lay nhiet do tu temp_Register
	return temp;
}

#endif
