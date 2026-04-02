/*
 * lunautils.h
 *
 *  Created on: Mar 15, 2025
 *      Author: strongp
 */

#ifndef INC_LUNAUTILS_H_
#define INC_LUNAUTILS_H_

#include "stdint.h"
#include "stdbool.h"

void U32ToU8x4(uint32_t val, uint8_t* res);
void FloatToU8x4(float val, uint8_t* res);
uint32_t U8x4ToU32(uint8_t* data);
float U8x4ToFloat(uint8_t* data);

#endif /* INC_LUNAUTILS_H_ */
