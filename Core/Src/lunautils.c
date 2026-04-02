/*
 * lunautils.c
 *
 *  Created on: Mar 15, 2025
 *      Author: strongp
 */
#include "lunautils.h"

void U32ToU8x4(uint32_t val, uint8_t* res){
	for(int i = 0; i < 4; i++){

		*(res+ i) = (((0x000000FF << (i * 8)) & val) >> (i * 8));
		//	*(res+ i) = (val >> (8 * i)) & 0xFF;
	}
}

uint32_t U8x4ToU32(uint8_t* data){
  uint32_t ret = 0;
  for(int i = 0;i<4;i++){
      ret = ret | ((uint32_t)(*(data+i))<<(i*8));
  }
  return ret;
}

float U8x4ToFloat(uint8_t* data){
	uint32_t val = 0;
	for(int i = 0;i<4;i++){
		val = val | ((uint32_t)(*(data+i))<<(i*8));
	}
	float ret = *(float*) &val;
	return ret;
}

void FloatToU8x4(float val, uint8_t* res){
	uint32_t int_val = *(uint32_t*) &val;
	for(int i = 0; i < 4; i++){

		*(res+ i) = (int_val >> (i * 8)) & 0xFF;
	}
}
