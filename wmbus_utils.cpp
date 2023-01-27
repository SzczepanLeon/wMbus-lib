/***********************************************************************************
    Filename: wmbus_utils.cpp
***********************************************************************************/

#include"aes.hpp"
#include"wmbus_utils.hpp"

#include<assert.h>
/**************
#include<memory.h>
**************/
#include<vector>

namespace std
{

bool decrypt_TPL_AES_CBC_IV(vector<unsigned char> &frame,
                            vector<unsigned char>::iterator &pos,
                            vector<unsigned char> &key,
                            unsigned char *iv,
                            int *num_encrypted_bytes,
                            int *num_not_encrypted_at_end) {
  vector<unsigned char> buffer;
  buffer.insert(buffer.end(), pos, frame.end());
  size_t num_bytes_to_decrypt = frame.end()-pos;
  uint8_t tpl_num_encr_blocks = 3; //t->tpl_num_encr_blocks

  if (tpl_num_encr_blocks) {
    num_bytes_to_decrypt = tpl_num_encr_blocks*16;
  }

  *num_encrypted_bytes = num_bytes_to_decrypt;
  *num_not_encrypted_at_end = buffer.size()-num_bytes_to_decrypt;

  if (key.size() == 0) {
    return false;
  }

  if (buffer.size() < num_bytes_to_decrypt) {
    num_bytes_to_decrypt = buffer.size();
  }

  // The content should be a multiple of 16 since we are using AES CBC mode.
  if (num_bytes_to_decrypt % 16 != 0) {
    assert (num_bytes_to_decrypt % 16 == 0);
  }

  unsigned char buffer_data[num_bytes_to_decrypt];
  memcpy(buffer_data, safeButUnsafeVectorPtr(buffer), num_bytes_to_decrypt);
  unsigned char decrypted_data[num_bytes_to_decrypt];
  memcpy(decrypted_data, buffer_data,num_bytes_to_decrypt);

  struct AES_ctx ctx;
  AES_init_ctx_iv(&ctx, &key[0], iv);

  AES_CBC_decrypt_buffer(&ctx, decrypted_data, num_bytes_to_decrypt);

  // Remove the encrypted bytes.
  frame.erase(pos, frame.end());

  // Insert the decrypted bytes.
  //frame.insert(frame.end(), decrypted_data, decrypted_data+num_bytes_to_decrypt);
  frame.insert(frame.end(), decrypted_data, decrypted_data+num_bytes_to_decrypt);

  if (num_bytes_to_decrypt < buffer.size()) {
    frame.insert(frame.end(), buffer.begin()+num_bytes_to_decrypt, buffer.end());
  }
  return true;
}

}// namespace std
