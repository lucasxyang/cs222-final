#include "StorageVolumes.h"
#include "printf.h"
configuration ThresholdAppC{
}
implementation {
  components MainC, ThresholdPrintC, LedsC,UniformQuantizerM;
  components PrintfC;
  components new TimerMilliC() as Timer;

  components new BlockStorageC(VOLUME_LOGTEST);
  ThresholdPrintC.Boot-> MainC;
  ThresholdPrintC.Leds-> LedsC;
  ThresholdPrintC.Timer->Timer;
  ThresholdPrintC.BlockRead->BlockStorageC;
  ThresholdPrintC.UniformQuantizer->UniformQuantizerM;
  ThresholdPrintC.PrintfControl-> PrintfC;
  ThresholdPrintC.PrintfFlush-> PrintfC;
  UniformQuantizerM.PrintfFlush->PrintfC;
  UniformQuantizerM.PrintfControl->PrintfC;
}

