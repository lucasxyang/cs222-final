#ifndef RUNLENGTH_H
#define RUN_LENGTH_H

enum {
  UBUF_EMPTY=20,
  CBUF_EMPTY=21,
  BITSTREAM_EMPTY=22
};

typedef struct _rlebuffer {
  uint16_t *buf;
  uint16_t length;
  uint16_t index;
} rlebuffer_t;

/* each */
typedef struct _entry {
  uint16_t val;
  uint8_t nbits;
} entry_t;

typedef struct _runlength_t {
  entry_t last_sample; // to detect run
  entry_t run_symbol;
  entry_t run_length; // the length of the current, unfinished run
  uint16_t max_run_length; // set this according to your quantization policy.
} runlength_t;

#endif
