#include"printf.h"
module UniformQuantizerM {
  provides interface UniformQuantizer;

  uses interface SplitControl as PrintfControl;
  uses interface PrintfFlush;

}
implementation {
   
  uquantizer_t *r ;
  inline command uquantizer_t * UniformQuantizer.updateQ (uquantizer_t *Q, int16_t *buf, uint16_t length) {
    uint16_t i;
    for (i=0;i<length;i++) {
      if (buf[i]<Q->min_sample) Q->min_sample = buf[i];
      if (buf[i]>Q->max_sample) Q->max_sample = buf[i];
    }
   // call PrintfControl.start();
      return Q;
  }

  /* Construct a nbit uniform quantizer structure and return to
     user. Only 4 and 8 bit currently supported. */
  command uquantizer_t*  UniformQuantizer.constructQuantizer (uquantizer_t *Q, uint8_t nbit) {
    int16_t diff = Q->max_sample - Q->min_sample;

    /* if the difference is less than the number of bins, then bin
       width = 1 */
    if (diff < (1<<nbit)) Q->bin_width = 1;
    else Q->bin_width = diff>>nbit;

    Q->nbit = nbit;
    return Q;
//    call PrintfControl.start();
    
  }

  /* given the uniform quantization bins, find the bin where the
     sample falls, and send the bin-index. The function returns
     uint16_t type, which can be stored into whichever format (4bit,
     8bit) the calling function requires. */
  inline command uint16_t UniformQuantizer.quantizeSample (int16_t sample, uquantizer_t *Q) {
    return ((uint16_t)((sample - Q->min_sample)/Q->bin_width));
//    call PrintfControl.start();
  }

  /* The input and output buffers (ubuf and qbuf resp) are of type
     strictly int16_t. len is the number if samples that need to be
     quantized. Both buffers need to be of appropriate length. */
  command uint16_t* UniformQuantizer.quantizeBuffer (int16_t *ubuf, uint16_t *qbuf,uint16_t len, uquantizer_t *Q) {
    uint16_t i;

    for (i=0;i<len;i++)
      qbuf[i] = (uint16_t)((ubuf[i] - Q->min_sample)/Q->bin_width);
      return qbuf;
//      call PrintfControl.start();
  }

  /* The input buffer (ubuf) is of type strictly int16_t. The output
     is a bitstream. len is the number if samples that need to be
     quantized. Both buffers need to be of appropriate length ie ubuf
     is of length 'len*2'bytes and qstream->buf is of length
     'len*Q->nbit' */
  event void PrintfControl.startDone(error_t err)
{
   printf("hello i am here"); 
   printf("binwidth : %d",r->bin_width);
   printf("nbit :%d",r->nbit);
  call   PrintfFlush.flush();

}
  event void PrintfFlush.flushDone(error_t err)
{
}
event void PrintfControl.stopDone(error_t err)
{
} 
}

