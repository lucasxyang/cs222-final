#include "StorageVolumes.h"
#include "printf.h"
configuration ThAppC{
}
implementation {
  components MainC, ThresholdPrintC, LedsC,ThresholdM;
  components PrintfC;
  components new TimerMilliC() as Timer;
  components new BlockStorageC(VOLUME_LOGTEST);

  ThresholdPrintC.Boot-> MainC;
  ThresholdPrintC.Leds-> LedsC;
  ThresholdPrintC.Threshold->ThresholdM;
  ThresholdPrintC.BlockRead->BlockStorageC;
  ThresholdPrintC.PrintfControl-> PrintfC;
  ThresholdPrintC.PrintfFlush-> PrintfC;
  ThresholdM.PrintfFlush->PrintfC;
  ThresholdM.PrintfControl->PrintfC;
  ThresholdPrintC.Timer->Timer;
}

