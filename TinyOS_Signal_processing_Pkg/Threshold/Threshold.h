typedef struct _histogram_t {

  uint16_t nbins; // number of bins

  /* pointer to an array of bins. number of entries = nbins + 1 (due
     to boundaries) */
  uint16_t *bin;

  /* array of size nbins + 1 that represent the edges. the edges is
     typically the spread of the signal, but need not be the case. If
     a sample is outside the two edges, it is discarded. */
  uint16_t *bin_edge; 
} histogram_t;

