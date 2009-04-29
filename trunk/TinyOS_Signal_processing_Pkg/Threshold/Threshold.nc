interface Threshold {
  
  /* use a single threshold for the entire data buffer */
  command int16_t ThresholdSample (int16_t sample, uint16_t threshold);
  
  /* use a single threshold for the entire data buffer */
  command error_t  UniformThreshold (int16_t *data, uint16_t length, uint16_t threshold);

  /* specify a different threshold for each resolution. Length
     corresponds to the length of both L and threshold buffers */
  command error_t NonUniformThreshold (int16_t *data, uint16_t *L, uint16_t *threshold, uint16_t length);
}

