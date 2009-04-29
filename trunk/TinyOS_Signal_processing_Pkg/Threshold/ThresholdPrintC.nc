#define SIZE 4000
#include "Timer.h"
#include "printf.h"
#include "Threshold.h"
module ThresholdPrintC {
  uses {
    interface Boot;  
    interface Leds;
    interface SplitControl as PrintfControl;
    interface PrintfFlush;
    interface Threshold;
    interface Timer<TMilli>;
    interface BlockRead;
  }
}
implementation {
int i;  
int16_t sample=4;
uint16_t threshold =2;
int16_t result;
uint32_t addr;
//int16_t arr[5]={3,-5,2,-4,7};
int16_t arr[SIZE];
uint16_t start_time,end_time;
uint16_t time_taken;
int count,NUMR;	
 event void Boot.booted()
	 {
        count=0;
        NUMR=20;
       // for(i=0;i<SIZE;i++)
       // arr[i]=i;
        start_time=call Timer.getNow();
        //result= call Threshold.ThresholdSample (sample,threshold);
        //call Threshold.UniformThreshold(arr,SIZE,threshold);
          call BlockRead.read(addr,arr,sizeof(arr));
        //call PrintfControl.start();
  	}
    
  event void PrintfControl.startDone(error_t error) {
        printf("i am taking %d ms to process",time_taken);
      //  printf("%d\n",result);
      //  for(i=0;i<SIZE;i++)
     //   printf("%d\n",arr[i]);
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
                        time_taken=end_time-start_time;
                        call PrintfControl.start();
                        }
                        else
                        {
                          call Threshold.UniformThreshold(arr,SIZE,threshold);
                          call BlockRead.read(addr,arr,sizeof(arr));
                        }
                }
        }


  event void PrintfControl.stopDone(error_t error) {
  }
  
  event void PrintfFlush.flushDone(error_t error) {
call PrintfControl.stop();
  }
  event void Timer.fired()
{
}
event void BlockRead.computeCrcDone(storage_addr_t x, storage_len_t y, uint16_t z, error_t result){}
}

