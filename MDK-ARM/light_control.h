/**
  ******************************************************************************
  * @file    light_control.h
  * @author  Colson Yang
  * @version V1.0
  * @date    30-March-2017
  * @brief   Analysis sensors' data, control lamps. 
  ******************************************************************************
*/

#ifndef __light_control_H
#define __light_control_H

#define LightNumMax 9
#define LightShowMax 2
#define LightShowTh 1
#define SensorNumMax 11

//Deqi
#define MY_FRMHDR     0x2B //0xA5
#define MY_FRMHDR1    0x4E
#define MY_FRMHDR2    0x4F
#define MY_FRMTIFIS   0x0D
#define MY_FRATILST   0x0A

#define IR_NUMS   40
#define DIS_NUM   0x41


#define IR_LEN   8//64
#define DIS_LEN  2


#define ADDELEN  15

			
unsigned char HumanCalcula(unsigned short ID,unsigned short SpecialPoint);

#endif
