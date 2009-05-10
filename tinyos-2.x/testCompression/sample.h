#ifndef SAMPLE_H
#define SAMPLE_H

#define SAMPLE_MSG_LENGTH   30
typedef nx_struct sample_msg {
  nx_uint8_t buffer[SAMPLE_MSG_LENGTH];
} sample_msg_t;

enum {
  AM_SAMPLE_MSG = 10,
};

#endif
