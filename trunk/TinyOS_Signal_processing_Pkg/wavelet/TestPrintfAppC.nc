#include "printf.h"
#include "Transform.h"
#include "StorageVolumes.h"

configuration TestPrintfAppC{
}
implementation {
  components MainC, TestPrintfC, LedsC,Transform1dM;
  components PrintfC;
  components new TimerMilliC() as Timer;
  components new BlockStorageC(VOLUME_LOGTEST);


  TestPrintfC.Boot -> MainC;
  TestPrintfC.Leds -> LedsC;
  TestPrintfC.BlockRead->BlockStorageC;
  TestPrintfC.WaveDecompose->Transform1dM;
  TestPrintfC.PrintfControl -> PrintfC;
  TestPrintfC.PrintfFlush -> PrintfC;
  TestPrintfC.Timer->Timer;
  Transform1dM.Leds->LedsC;
  Transform1dM.PrintfFlush->PrintfC;
  Transform1dM.PrintfControl->PrintfC;
  Transform1dM.Timer->Timer;
}

