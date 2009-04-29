#include "printf.h"
#include "Transform.h"
#define SIZE 128
module TestPrintfC {
  uses {
    interface Boot;  
    interface WaveDecompose;
    interface Leds;
    interface SplitControl as PrintfControl;
    interface PrintfFlush;
    interface BlockRead;
    interface Timer<TMilli>;

  }
}
implementation {
	
 
//  int16_t bufin[24]={1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8};
    uint8_t j;
    int16_t bufin[SIZE];
//  int16_t bufout[24];
    int16_t bufout[SIZE];
    uint32_t addr,len;   
    uint8_t i=0;
    uint8_t NUMR;
    error_t result;
    int count;
    uint16_t start_time,end_time;
    uint16_t time_taken;

  event void Boot.booted() {
      addr=0x00;
      NUMR=20;
      count=0;
      call BlockRead.read(addr,bufin,sizeof(bufin));
      call PrintfControl.start();     
  }
  event void PrintfControl.startDone(error_t error) {
        printf("Hello ! i am writing to you");
        printf("Time taken : %d ms",time_taken);
  	call PrintfFlush.flush();
  }

  event void PrintfControl.stopDone(error_t error) {
  }
  
 event void PrintfFlush.flushDone(error_t error) {
call PrintfControl.stop();
call Leds.led1Toggle();
  }

  event error_t WaveDecompose.decomposeDone(error_t success)
{
  if(success==SUCCESS)
{
   printf("hello");
      
   return SUCCESS;
}
}
event void BlockRead.readDone(storage_addr_t addr1, void* buf1, storage_len_t len1, error_t error)
        {
                if (error == SUCCESS)
                {       
                        addr = addr + sizeof(bufin);
                        if(++count==NUMR)
                        {
                         end_time=call Timer.getNow();
                         time_taken=end_time-start_time;
                         call PrintfControl.start();
                        }
                        else
                        {
                          call WaveDecompose.decompose(22,bufin,bufout);
                          call BlockRead.read(addr,bufin,sizeof(bufin));
                        }
                }
        }


event void BlockRead.computeCrcDone(storage_addr_t x, storage_len_t y, uint16_t z, error_t result){}
event void Timer.fired()
{
}

}
