#include "RunLength.h"

interface RunLength {
  command runlength_t* initState(runlength_t *s, uint16_t run_symbol, uint8_t run_symbol_bits, 
				   uint8_t sample_bits, uint8_t run_length_bits);
  command error_t addSampleToBuffer(uint16_t data, rlebuffer_t *rbuf, runlength_t *state);
  command error_t addBufferToBuffer(rlebuffer_t *ubuf, rlebuffer_t *cbuf, runlength_t *state);
}

