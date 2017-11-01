/**
  ******************************************************************************
  * @file    light_control.c
  * @author  Colson Yang
  * @version V1.0
  * @date    30-March-2017
  * @brief   Analysis sensors' data, control lamps. 
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "light_control.h"
#include <stdint.h>
#include <stdbool.h>
/* Define property structure -------------------------------------------------*/

/******************************灯具-传感器 对应表********************************/
/************灯Id 传感器id1、2、3 传感器点1、2、3 权值百分比1、2、3**************/
unsigned char LightSensorTable[9][16] =
{
	/************* 顶灯 **************/
	1,1,2,3,4,5,24,24,24,24,24,18,55,18,18,0,
	2,2,3,4,5,6,24,24,24,24,24,18,18,55,18,18,
	3,4,5,6,7,8,24,24,24,24,24,18,18,55,18,18,
	4,6,7,8,9,10,24,24,24,24,24,18,18,55,18,18,
	5,7,8,9,10,11,24,24,24,24,24,0,18,18,55,18,
	/************* 射灯 **************/
	6,1,2,3,4,5,24,24,24,24,24,30,60,30,0,0,
	7,4,5,6,7,8,24,24,24,24,24,30,60,30,0,0,
	8,7,8,9,10,11,24,24,24,24,24,30,60,30,0,0,
	9,9,10,11,7,8,24,24,24,24,24,30,60,30,0,0,
};

/*******************************顶灯-射灯/台灯 对应表*********************************/
unsigned char LightLightTable[5][3] =
{
	1,6,0,
	2,6,7,
	3,7,8,
	4,8,0,
	5,9,0
};

/*******************************灯具输出 对应表*********************************/
//uint8_t LightOutputTable[47] =
//{
//	0xA5,0x2F,
//	0x30,0x30,0x30,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//	0x00,
//	0x55,0xAA
//};

uint8_t LightOutputTable[20] =
{
	0x41,0x54,0x2B,0x4E,0x4F,0x54,0x49,0x46,0x59,0x3D,
	0x01,0x50,0x50,0x50,0x50,0x50,0x00,0x00,0x00,0x00
};

//unsigned char HumanTable[3][64] = {0};
extern bool SensorData[SensorNumMax][64];

unsigned char HumanCalcula(unsigned short ID,unsigned short SpecialPoint)
{
	unsigned char i = 0;
	char RowI = 0;
	char RowX = 0;
	char ColI = 0;
	char ColX = 0;
	unsigned char Gear4 = 0;
	unsigned char Gear3 = 0;
	unsigned char Gear2 = 0;
	unsigned char Gear1 = 0;
	unsigned char Gear = 0;
	
	RowX = SpecialPoint / 16;
	ColX = SpecialPoint % 16;
	
	for(i = 0; i < 64; i++ )
	{
		RowI = i / 16;
		ColI = i % 16;
		if( ((RowI - RowX) < 2) && ((RowI - RowX) > -2) && ((ColI - ColX) < 2) && ((ColI - ColX) > -2))
			//		if( ((RowI - RowX) < 2) && ((RowI - RowX) > -2) && ((ColI - ColX) < 2) && ((ColI - ColX) > -2))
		{
			if( SensorData[ID-1][i] == 1)
			{
				Gear4 = 1;
				goto CaL;
			}
		}
		else if( ((RowI - RowX) < 3) && ((RowI - RowX) > -3) && ((ColI - ColX) < 3) && ((ColI - ColX) > -3))
		{
			if( SensorData[ID-1][i] == 1)
			{
				Gear3 = 1;
			}
		}
//		else if( ((ColI - ColX) < 7) || ((ColI - ColX) > -7))
		else if( ((RowI - RowX) < 4) && ((RowI - RowX) > -4) && ((ColI - ColX) < 4) && ((ColI - ColX) > -4))
		{
			if( SensorData[ID-1][i] == 1)
			{
				Gear2 = 1;
			}
		}
		else 
		{
			if( SensorData[ID-1][i] == 1)
			{
				Gear1 = 1;
			}
		}
	}
	CaL:
	if(Gear4 == 1)
		Gear = 4;
	else if(Gear3 == 1)
		Gear = 3;
	else if(Gear2 == 1)
		Gear = 2;
	else if(Gear1 == 1)
		Gear = 1;
	else
		Gear = 0;

	return Gear;
}

/**************************************主函数**************************************/
/*
int main(void)
{
	unsigned short i,j;
	while(1)
	{
		for(i = 0;i < LightNumMax;i++)
		{
			LightOutputTable[i] = ( HumanCalcula(LightSensorTable[i][1],LightSensorTable[i][4]) * LightSensorTable[i][7] 
														+ HumanCalcula(LightSensorTable[i][2],LightSensorTable[i][5]) * LightSensorTable[i][8] 
														+ HumanCalcula(LightSensorTable[i][3],LightSensorTable[i][6]) * LightSensorTable[i][9] )/100;
		}
		for(i = 0;i < LightShowMax;i++)
		{
			j = LightLightTable[i][0] % 100;
			LightOutputTable[j] =  LightOutputTable[j] - LightShowTh * LightOutputTable[LightLightTable[i][1]%100] 
																								 - LightShowTh * LightOutputTable[LightLightTable[i][2]%100];
			if(LightOutputTable[j] < 0)
			{
				LightOutputTable[j] = 0;
			}
		}
	}
}
*/

/************************ (C) COPYRIGHT WHU EIS INTERNET&IT LAB *****END OF FILE****/
