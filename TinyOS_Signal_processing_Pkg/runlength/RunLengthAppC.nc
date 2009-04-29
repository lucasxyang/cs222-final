#include "StorageVolumes.h"
configuration RunLengthAppC
{
}
implementation
{
  components MainC, RunLengthC, LedsC , RunLengthM, PrintfC ;
  components new TimerMilliC() as Timer;
components new BlockStorageC(VOLUME_LOGTEST);

  RunLengthC -> MainC.Boot;
  RunLengthC.Leds -> LedsC;
  RunLengthC.RunLength ->RunLengthM;
  RunLengthC.PrintfControl->PrintfC;
  RunLengthC.PrintfFlush->PrintfC;
  RunLengthC.BlockRead->BlockStorageC;
  RunLengthM.PrintfControl->PrintfC;
  RunLengthM.PrintfFlush->PrintfC;
  RunLengthM.Timer->Timer;
  RunLengthC.Timer->Timer;
}

