#include "printf.h"
module ThresholdM {
  uses interface SplitControl as PrintfControl;
  uses interface PrintfFlush;
  provides interface Threshold;
}
implementation {
 int16_t output;
 int16_t *data1;
 uint16_t start_time,end_time;
int g;
  /* use a single threshold for the entire data buffer */
//  call PrintfControl.start();
  command int16_t Threshold.ThresholdSample (int16_t sample, uint16_t threshold) {
     if (abs(sample) > threshold)
      output= sample;
    else
      output=0;
     return output;
  }

  /* use a single threshold for the entire data buffer */
  command error_t Threshold.UniformThreshold (int16_t *data, uint16_t length, uint16_t threshold) {
    int i;
    for (i=0;i<length;i++) {
      if (abs(data[i]) > threshold)
	data[i]=data[i];
      else
	data[i]=0;
    }
    data1=data;
//    call PrintfControl.start();
    return SUCCESS;
      
  }

  /* specify a different threshold for each resolution. Length
     corresponds to the length of both L and threshold buffers */
  command error_t Threshold.NonUniformThreshold (int16_t *data, uint16_t* L, uint16_t* threshold, uint16_t length) {
    int i;
    int j;
    uint16_t previ=0;

    for (i=0; i<length; i++) {
      for (j=previ; j<previ+L[i]; j++) {
	if (abs(data[j]) > threshold[i])
	  data[j]=output;
	else
	  data[j]=0;
      }
      previ += L[i];
    }    

    return SUCCESS;
  }
  event void PrintfControl.startDone(error_t err)
{
}
event void PrintfFlush.flushDone(error_t err)
{
}

event void PrintfControl.stopDone(error_t err)
{
}
}

