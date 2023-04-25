/***********************************************************************************
    Filename: utils.hpp
***********************************************************************************/

#ifndef _UTILS_H
#define _UTILS_H

#include <Arduino.h>
#include <stdint.h>
#include <vector>

//----------------------------------------------------------------------------------
//  Function Declareration
//----------------------------------------------------------------------------------
void dumpHex(uint8_t* data, uint8_t len, bool newLine = true);
unsigned char *safeButUnsafeVectorPtr(std::vector<unsigned char> &v);
bool decrypt_TPL_AES_CBC_IV(vector<unsigned char> &frame, vector<unsigned char>::iterator &pos,
                            vector<unsigned char> &key, unsigned char *iv,
                            int *num_encrypted_bytes,
                            int *num_not_encrypted_at_end);

#endif