#include "StorageVolumes.h"
configuration BlockWRAppC
{
}

implementation
{
components MainC,BlockWR,PrintfC;
components new TimerMilliC() as Timer;
components new BlockStorageC(VOLUME_LOGTEST);
BlockWR.Boot->MainC;
BlockWR.Timer->Timer;
BlockWR.BlockWrite->BlockStorageC;
BlockWR.BlockRead->BlockStorageC;
BlockWR.PrintfControl->PrintfC;
BlockWR.PrintfFlush->PrintfC;
//BlockWR.Leds->LedsC;

}


