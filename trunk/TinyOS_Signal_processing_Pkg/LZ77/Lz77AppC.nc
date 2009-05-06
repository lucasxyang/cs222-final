#include "printf.h"
//#include "time.h"
#include "StorageVolumes.h"

configuration Lz77AppC
{
//  provides interface Lz77Compression;
}

implementation
{
components LZ77M,MainC,TestC,LedsC;
components new TimerMilliC() as Timer;
components new BlockStorageC(VOLUME_LOGTEST);

TestC.Boot->MainC;
TestC.Timer->Timer;
LZ77M.Timer->Timer;
TestC.Leds->LedsC;

TestC.Lz77Compression->LZ77M.Lz77Compression;
TestC.BlockRead->BlockStorageC;
}
