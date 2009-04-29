#define SIZE 256
#include "Timer.h"
#include "printf.h"
module BlockWR
{
  uses interface BlockWrite;
  uses interface BlockRead;
  uses interface Timer<TMilli>;
  uses interface SplitControl as PrintfControl;
  uses interface PrintfFlush;
  uses interface Boot;
}
implementation
{
  bool read_busy =FALSE;
  uint32_t addr,len;
  uint16_t start_time,end_time;
  uint16_t time_taken;
  int16_t buf[SIZE];
  int16_t out[SIZE];
  int32_t arr[110];
  int8_t NUMWR,NUMR;
  int i,count;

event void Boot.booted()
{    
     // addr=0x00;
       NUMWR=400;
       // NUMR=110;
        count=0;
         i=0;
   //for(i=0;i<256;i++)
  //  space=call BlockRead.getSize();
// call PrintfControl.start();
     for(i=0;i<SIZE;i++)
     buf[i]=7;
    call BlockWrite.erase();
   // call BlockRead.read(addr,out,sizeof(buf));

}event void  PrintfFlush.flushDone(error_t err)
{
}
event void Timer.fired()
{
}

event void BlockWrite.eraseDone(error_t err)
        {
        // call PrintfControl.start();
                if (err == SUCCESS)
                {
                    addr=0x00;
                 
                  call BlockWrite.write(addr, buf, sizeof(buf));
                }
         }

event void BlockWrite.writeDone(storage_addr_t addr1, void* buf1, storage_len_t len1, error_t error)
        {
                if (error == SUCCESS)
                {      // call BlockWrite.sync();
                        addr = addr + sizeof(buf);
                       if (++count==NUMWR)
                        call BlockWrite.sync();
                        else
                        call BlockWrite.write(addr,buf,sizeof(buf));
                }
             call PrintfControl.start();
        }
	event void BlockRead.readDone(storage_addr_t addr1, void* buf1, storage_len_t len1, error_t error)
        {
                if (error == SUCCESS)
                {       
                        addr = addr + sizeof(buf);
                        if(++count==NUMR)
                        call PrintfControl.start();
                        else
                        {
                        arr[i++]=addr;
                        call BlockRead.read(addr,out,sizeof(buf));
                        }
                }
        }

event void PrintfControl.startDone(error_t err)
{
   printf("hello i am writing to you");
  // printf("space %d",space);
   for(i=0;i<32;i++)
   printf("%d",out[i]);
   printf("\n");
   for(i=0;i<110;i++)
   printf("%d : ",arr[i]);
   call PrintfFlush.flush();
}

event void PrintfControl.stopDone(error_t err)
{

}

 event void BlockWrite.syncDone(error_t result){}
 event void BlockRead.computeCrcDone(storage_addr_t x, storage_len_t y, uint16_t z, error_t result){}

}

