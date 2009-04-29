#include "Transform.h"
interface WaveDecompose
{
  command error_t decompose (uint8_t kernel, int16_t *buffer_in, int16_t *buffer_out);

  event error_t decomposeDone(error_t success);
  
  command void setMode(uint8_t mode_in);
}
