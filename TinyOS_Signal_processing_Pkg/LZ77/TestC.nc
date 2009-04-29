module TestC

{
  uses interface Lz77Compression;
  uses interface SplitControl as PrintfControl;
  uses interface PrintfFlush;
  uses interface Boot;
  uses interface BlockRead;
  uses interface Timer<TMilli>;
  uses interface Leds;
}

implementation
{
int i;
int buf[SIZE];
int arr[2];
int NUMR;
uint32_t addr,len;
int count;
uint64_t start_time,end_time;
uint64_t time_taken;
event void Boot.booted()
{
     i=0;
     count=0;
     NUMR=3;
     addr=0x00;
     start_time=call Timer.getNow();
    // call PrintfControl.start();

     call BlockRead.read(addr,buf,sizeof(buf));
}
event void  PrintfControl.startDone(error_t err)
{
 printf("hello i am writing to you");
 printf("I am taking : %u ms\n",time_taken);
 for(i=0;i<2;i++)
 printf("%d\n",arr[i]);
 call PrintfFlush.flush();
}
event void PrintfControl.stopDone(error_t err)
{
}

event void BlockRead.readDone(storage_addr_t addr1, void* buf1, storage_len_t len1, error_t error)
        {
                if (error == SUCCESS)
                {       
                        addr = addr + sizeof(buf);
                        arr[i++]=addr;
                        if(++count==NUMR)
                        {
                        end_time=call Timer.getNow();
                        call Leds.led0Toggle();
                        time_taken=end_time-start_time;
                        call PrintfControl.start();
                        }
                        else
                        {
                        call Lz77Compression.Lz77Encode(buf);
                     //   call BlockRead.read(addr,buf,sizeof(buf));
                        }
                }
        }


event void  PrintfFlush.flushDone(error_t err)
{
}
event void Timer.fired()
{
}
event error_t Lz77Compression.Lz77EncodeDone(error_t err)
{
 if (err==SUCCESS)
    call BlockRead.read(addr,buf,sizeof(buf));
}
event void BlockRead.computeCrcDone(storage_addr_t x, storage_len_t y, uint16_t z, error_t result){}
}
