#include "MyI2C.h"

void MyI2C_W_SCL(uint8_t BitValue)
{
	HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, (GPIO_PinState)BitValue);
    Delay_us(5);
}

void MyI2C_W_SDA(uint8_t BitValue)
{
	HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, (GPIO_PinState)BitValue);
    Delay_us(5);
}

uint8_t MyI2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin);
	return BitValue;
}

void MyI2C_Start(void)
{
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(0);
}

void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
		MyI2C_W_SDA(Byte & (0x80 >> i));
		MyI2C_W_SCL(1);
		MyI2C_W_SCL(0);
	}
}

uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t i, Byte = 0x00;
    MyI2C_W_SDA(1); // 释放SDA总线，准备接收数据
    for (i = 0; i < 8; i ++)
    {
        MyI2C_W_SCL(1);
        if (MyI2C_R_SDA() == 1){Byte |= (0x80 >> i);}
        MyI2C_W_SCL(0);
    }
    return Byte;
}

void MyI2C_SendAck(uint8_t AckBit)
{
	MyI2C_W_SDA(AckBit);
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);
}

uint8_t MyI2C_ReceiveAck(void)
{
	uint8_t AckBit;
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	AckBit = MyI2C_R_SDA();
	MyI2C_W_SCL(0);
	return AckBit;
}

void I2C_WriteReg(uint8_t address, uint8_t reg, uint8_t data)
{
    MyI2C_Start();
    MyI2C_SendByte(address);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(data);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

uint16_t I2C_ReadReg(uint8_t address, uint8_t reg)
{
    uint8_t data_high, data_low;
    uint16_t data;
    
    MyI2C_Start();
    MyI2C_SendByte(address);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);
    MyI2C_ReceiveAck();
   
    MyI2C_Start();
    MyI2C_SendByte(address | 0x01);
    MyI2C_ReceiveAck();
    data_low = MyI2C_ReceiveByte(); // 读取第一个字节（数据低8位）
    MyI2C_SendAck(0);               // 发送ACK，表示将继续读取
    data_high = MyI2C_ReceiveByte(); // 读取第二个字节（数据高8位）
    MyI2C_SendAck(1); 
    
    MyI2C_Stop();
    
    data = (uint16_t)(data_high << 8) | data_low;
    return data;
}
