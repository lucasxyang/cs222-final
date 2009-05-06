#include "Sampling.h"

configuration TestCompressionC 
{
}
implementation 
{
    components MainC, new TestCompressionP(uint16_t), LedsC;

    TestCompressonP.Boot -> MainC;
    TestCompressonP.Leds -> LedsC;

    components new TimerMilliC();
    TestCompressonP.Timer -> TimerMilliC;

    components new SamplingC(uint16_t);
    TestCompressionP.Sampling -> SamplingC;
}



