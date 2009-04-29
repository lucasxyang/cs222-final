#include "printf.h"
#include "Timer.h"
#define SIZE 256
module RunLengthC
{

  uses interface Leds;
  uses interface Boot;
  uses interface RunLength;
  uses interface SplitControl as PrintfControl;
  uses interface PrintfFlush;
  uses interface BlockRead;
  uses interface Timer<TMilli>;
}
implementation
{

  runlength_t *rl,*rrl,t;
   rlebuffer_t * rlb,rls;
  uint16_t run_symbol;
  uint8_t run_symbol_bits;
  uint8_t sample_bits;
  uint8_t run_length_bits;
  uint16_t data;
  uint16_t arr[50];
  uint32_t addr,len;
  int8_t NUMR,count;
  uint32_t start_time,end_time;
  uint32_t time_taken;
    
//  uint16_t data1[10]={1,2,2,2,2,2,5,6,5,6};
  uint16_t data1[SIZE];
  int i;
  event void Boot.booted()
  {
  NUMR=200;
  count=0;
  rl=&t;
  rlb=&rls;
  
  run_length_bits=0;
  run_symbol=1000;
  sample_bits=4;
  run_length_bits=8;
  rls.index=0;
  rls.length=50;
  rls.buf=arr;
  start_time=call Timer.getNow();
  rrl= call RunLength.initState(rl,run_symbol,run_symbol_bits,sample_bits,run_length_bits);
   call BlockRead.read(addr,data1,sizeof(data1));
  }
  event void PrintfControl.startDone(error_t error) {

      printf("hello ,i am taking %d ms",time_taken);

  }

  event void PrintfControl.stopDone(error_t error) {
  }
  
  event void BlockRead.readDone(storage_addr_t addr1, void* buf1, storage_len_t len1, error_t error)
        {
                if (error == SUCCESS)
                {       
                        addr = addr + sizeof(data1);
                        if(++count==NUMR)
                          { 
                                end_time=call Timer.getNow();
                                time_taken=end_time-start_time;
                                call PrintfControl.start();
                          }
                        else
                        {
                         for (i=0;i<SIZE;i++)
                            {
                               data=data1[i];
                               call RunLength.addSampleToBuffer(data,rlb,rrl);
                            }

                        call BlockRead.read(addr,data1,sizeof(data1));
                        }
                }
        }


 event void PrintfFlush.flushDone(error_t error) {
call PrintfControl.stop();
call Leds.led1Toggle();
  }
 event void BlockRead.computeCrcDone(storage_addr_t x, storage_len_t y, uint16_t z, error_t result){}

event void Timer.fired()
{
}
}
