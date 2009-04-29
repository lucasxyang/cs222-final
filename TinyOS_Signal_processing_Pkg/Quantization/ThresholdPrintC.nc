#include "Timer.h"
#define SIZE 256
#include "printf.h"
#include "Quantizer.h"
module ThresholdPrintC {
  uses {
    interface Boot;  
    interface Leds;
    interface SplitControl as PrintfControl;
    interface PrintfFlush;
    interface UniformQuantizer;
    interface BlockRead;
    interface Timer<TMilli>;

   }
}
implementation {
 // int16_t arr[5]={-16,5,8,6,32};
  int16_t arr[SIZE];
  int i;
  uint16_t * brr;
  uquantizer_t t={4,7,2,0};
  uquantizer_t *r,*a,b;
  int16_t drr[6]={5,7,8,-9,4,6};
  int16_t data =6;
  uint16_t bin_index=0;
  uint16_t crr[6];
  uint32_t addr;
  int count,NUMR;
 //uint16_t *err;
  uint16_t timetaken;
  uint16_t start_time,end_time;

  

	
 event void Boot.booted()
	 {
      addr=0x00;
      NUMR=200;
      call BlockRead.read(addr,arr,sizeof(arr));
      start_time=call Timer.getNow();

      /*
	r= call UniformQuantizer.updateQ(&t,arr,5);
	a= call UniformQuantizer.constructQuantizer(r,4);
        bin_index= call UniformQuantizer.quantizeSample(data,a);
	call UniformQuantizer.quantizeBuffer(drr,crr,6,a);
	 call PrintfControl.start();
       */
  	}
    
  event void PrintfControl.startDone(error_t error) {
/*        printf("%d\n",a->bin_width);
        printf("%d\n",a->nbit);
        printf("%d\n",a->min_sample);
        printf("%d\n",a->max_sample);
        printf("%d\n",bin_index);
        for(i=0;i<6;i++)
        printf("%d\n",crr[i]);
*/      printf("time Taken : %d",timetaken);
       	call PrintfFlush.flush();
  }
  event void BlockRead.readDone(storage_addr_t addr1, void* buf1, storage_len_t len1, error_t error)
        {
                if (error == SUCCESS)
                {       
                        addr = addr + sizeof(arr);
                        if(++count==NUMR)
                        {
                        end_time=call Timer.getNow();
                        timetaken=end_time-start_time;

                        call PrintfControl.start();
                        }
                        else
                        {
                          r= call UniformQuantizer.updateQ(&t,arr,5);
	                    a= call UniformQuantizer.constructQuantizer(r,4);
                          bin_index= call UniformQuantizer.quantizeSample(data,a);
	                    call UniformQuantizer.quantizeBuffer(drr,crr,6,a);

                          
                          call BlockRead.read(addr,arr,sizeof(arr));
                        }
                }
        }


  event void PrintfControl.stopDone(error_t error) {
  }
  
  event void PrintfFlush.flushDone(error_t error) {
call PrintfControl.stop();
  }

event void BlockRead.computeCrcDone(storage_addr_t x, storage_len_t y, uint16_t z, error_t result){}

event void Timer.fired()
{
}


}

