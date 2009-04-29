interface UniformQuantizer {
  
  /* update the max and min values on uquantizer_t */
  command uquantizer_t * updateQ (uquantizer_t *Q, int16_t *buf, uint16_t length);

  /* Construct a nbit uniform quantizer structure and return to
     user. Only 4 and 8 bit quantizers are currently supported */
  command uquantizer_t * constructQuantizer (uquantizer_t *Q, uint8_t nbit);
  
  /* given the uniform quantization bins, find the bin where the
     sample falls, and send the bin-index */
  command uint16_t quantizeSample (int16_t sample, uquantizer_t *Q);

  /* Quantize the buffer 'ubuf' and place it in 'qbuf'. qbuf is packed
     according to the number of bits requested in the quantization */
  command uint16_t* quantizeBuffer (int16_t *ubuf, uint16_t *qbuf, 
			       uint16_t len, uquantizer_t *Q);


}

