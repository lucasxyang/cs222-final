#include "Timer.h"
module RunLengthM
{
  provides interface RunLength;
  uses interface SplitControl as PrintfControl;
  uses interface PrintfFlush;
  uses interface Timer<TMilli>;
}
implementation
{
 runlength_t *q, *s;
 int j;
 rlebuffer_t *rb;
 uint16_t start_time,end_time;
 uint16_t time_taken; 
#include "inet.h"
  
  command runlength_t * RunLength.initState(runlength_t *r, uint16_t run_symbol, uint8_t run_symbol_bits,uint8_t sample_bits, uint8_t run_length_bits) {
    start_time=call Timer.getNow();
    s=r;
    s->run_symbol.val = hton16(run_symbol);
    s->run_symbol.nbits = run_symbol_bits;

    s->run_length.val = 0;
    s->run_length.nbits = run_length_bits;

    s->last_sample.val = 0;
    s->last_sample.nbits = sample_bits;
                                      
    s->max_run_length = (1<<run_length_bits) - 1;
//    call PrintfControl.start();
       
    return s;
  }


  command error_t RunLength.addSampleToBuffer(uint16_t data, rlebuffer_t *rbuf, runlength_t *state) {
    uint16_t i;
    rb=rbuf;

    if ( (data==state->last_sample.val) && (state->run_length.val < state->max_run_length)) {
         state->run_length.val ++ ;
    } else { //end of a run of data
      if (state->run_length.val <= 3) { //no encoding if run_length <= 3
	
	while (state->run_length.val > 0) {
	  /* not enough space in cbuf to store samples. Note that length = index + 1 */
	  if (rbuf->length == rbuf->index)
	    return CBUF_EMPTY;

	  rbuf->buf[rbuf->index++] = state->last_sample.val;
	  state->run_length.val --;
	}
      } else { //RLE if run_length >3 
	
	if (rbuf->length - rbuf->index <= 3 ) { //fill the rest of the buffer 
	  for ( i=rbuf->index ; i < rbuf->length; i++ ) {
	    rbuf->buf[rbuf->index++] = state->last_sample.val;
	    state->run_length.val--;
	    //	    rbuf->buf[rbuf->index++] = 0xff;
	  }
	  return CBUF_EMPTY;
	} else {
	  //output the RLE code to buffer ( start with 0xaa 0xee )
	  rbuf->buf[rbuf->index++] = 0xaaee;
	  rbuf->buf[rbuf->index++] = state->run_length.val;
	  rbuf->buf[rbuf->index++] = state->last_sample.val;
	}
      } //if (run_length <=3)
      state->last_sample.val = data;
      state->run_length.val = 1;
    }
    return SUCCESS ;
    end_time=call Timer.getNow();
    time_taken=end_time-start_time;
  }

  command error_t RunLength.addBufferToBuffer(rlebuffer_t *ubuf, rlebuffer_t *cbuf, runlength_t *state) {
    uint16_t i;
    uint8_t status;

    /* add each sample till either the uncompressed or compressed
       buffer is empty */
    for (i=ubuf->index; i<ubuf->length; i++) {
      status = call RunLength.addSampleToBuffer (ubuf->buf[i], cbuf, state);
      if (status != SUCCESS) return status;
      ubuf->index = i+1;
    }
    return UBUF_EMPTY;
  }


 event void PrintfControl.startDone(error_t error) {
     printf("i am taking %d ms",time_taken);
     for(j=0;j<rb->index;j++)
     printf("%d",rb->buf[j]);
     call PrintfFlush.flush();
  }

  event void PrintfControl.stopDone(error_t error) {
  }
  event void PrintfFlush.flushDone(error_t err)
{
}
  event void Timer.fired()
{
}
}

