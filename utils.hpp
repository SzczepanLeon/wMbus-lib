/***********************************************************************************
    Filename: utils.hpp
***********************************************************************************/

#ifndef _UTILS_H
#define _UTILS_H

#include <Arduino.h>
#include <stdint.h>
#include<vector>

//----------------------------------------------------------------------------------
//  Function Declareration
//----------------------------------------------------------------------------------
void dumpHex(uint8_t* data, uint8_t len, bool newLine = true);
unsigned char *safeButUnsafeVectorPtr(std::vector<unsigned char> &v);

#endif