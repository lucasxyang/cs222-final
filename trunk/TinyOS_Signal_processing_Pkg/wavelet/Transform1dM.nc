#include "Timer.h"
#include "Transform.h"
#include "printf.h"


module Transform1dM {
  provides {
    interface WaveDecompose;
  }
  uses interface Leds;
  uses interface PrintfFlush;
  uses interface SplitControl as PrintfControl;
  uses interface Timer<TMilli>;
  }


implementation {

  uint8_t kernel,m,w;
  int16_t *buf_in, *buf_out;
  bool busy;
  uint8_t extension_mode=SYM; // by default, set to symmetric
			      // extension (boundary value symmetric
 int16_t *buf1,*coeff ;
 int16_t actual_length = TL;
 uint16_t start_time,end_time;
 uint16_t time_taken;
			      // replication)
  
  void add_padding(int16_t *buf); 
  
  void InPlaceTransform () {
    int k,s,l;
    int pos,j;

    /*  buf_in is of length at least 2^DEPTH */
      
    for (k=DEPTH; k>=1; k--) {
      s = 1<<(DEPTH-k);
      /*  first calculate d(i) */
      for (l = 0; l<=(LENGTH>>(DEPTH-k+1)) - 2;l++) {
	buf_in[((l<<1)+1)*s] = buf_in[((l<<1)+1)*s] -
	  ((buf_in[((l<<1))*s] + buf_in[((l<<1)+2)*s]) >> 1);
      }

      
      l=(LENGTH>>(DEPTH-k+1)) - 1;
      buf_in[((l<<1)+1)*s] = buf_in[((l<<1)+1)*s] 
	- ((buf_in[((l<<1))*s] + buf_in[((l<<1))*s]) >> 1);
      
      /*  now calculate all the s(i) */
      buf_in[0] = buf_in[0] + ((buf_in[s] + buf_in[s])>>2);
      
      for (l = 1; l<=(LENGTH>>(DEPTH-k+1)) - 1;l++) {
	buf_in[((l<<1))*s] = buf_in[((l<<1))*s]
	  + ((buf_in[((l<<1)-1)*s] + buf_in[((l<<1)+1)*s]) >> 2);
      }
    }
   

    /* re-sort transformed data to make it amenable for decomposition */
    pos = 0;
    buf1=&buf_out[PADDING];
    /* first get the coefficients */
    s = (1<<DEPTH);
      
    for (k=0; k<actual_length;k+=s) {
      buf1[pos] = buf_in[PADDING + k];
      pos++;
  //  call Leds.led1Toggle();
    }
    coeff=buf1;
   
    /* place each detail */
    for (l=DEPTH;l>=1;l--) {
      s= 1<<l;
      for (j=(1<<(l-1));j<actual_length;j+=s) {
	buf1[pos] = buf_in[PADDING + j];
	pos++;
      }
 }
    signal WaveDecompose.decomposeDone(SUCCESS);
    busy = FALSE;
    end_time=call Timer.getNow();
    time_taken=end_time-start_time;
    call  Leds.led0Toggle();
  }

event void PrintfControl.startDone(error_t error)
{
  printf("I am processing 256 bytes\n");
  printf("I am taking %d ms",time_taken);
  printf("\n Output :");
  for(w=0;w<SIZE;w++)
  printf("%d ",buf_out[w]);
  call PrintfFlush.flush();

}
event void PrintfControl.stopDone(error_t error)
{
}


  void task decomposeTask() {
    /*  Switch on the specified kernel type */
    switch (kernel) {
    case 22: 
      InPlaceTransform();
      break;
    }
  }
   


  /* both buffer_in and buffer_out should be exactly of length 2^DEPTH */
  command error_t WaveDecompose.decompose (uint8_t kernel_in, int16_t *buffer_in, int16_t *buffer_out) {
    start_time=call Timer.getNow();

    /* Is there a decompose task already queued */
    if (busy)
      return FAIL;
    else {

      busy = TRUE;
      kernel = kernel_in;
       buf_in = buffer_in;
      buf_out = buffer_out;

      /* add the required padding */
      add_padding(buf_in);
     
      /* post task to decompose buf_in */
      post decomposeTask();
  //    call PrintfControl.start();
    }
    return SUCCESS;
  }



  command void WaveDecompose.setMode (uint8_t mode_in) {
    extension_mode = mode_in;
  }

  void add_padding(int16_t *buf) {
    int i;

    switch (extension_mode) {
    case SYM:
      for (i=0;i<PADDING; i++)
	buf[i] = buf[PADDING];
      for (i=TL + PADDING;i<LENGTH; i++)
	buf[i] = buf[TL + PADDING - 1];
      break;
    case ZPD:
      for (i=0;i<PADDING; i++)
	buf[i] = 0;
      for (i=TL + PADDING;i<LENGTH; i++)
	buf[i] = 0;
      break;
    case EXP:
      break;
    }    
  }


event void PrintfFlush.flushDone(error_t error)
{
}
event void Timer.fired()
{
}
}
