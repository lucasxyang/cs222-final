#include "Timer.h"
#include "Sampling.h"
#include "PrintfUART.h"


generic module TestCompressionP(typedef sample_type)
{
    uses interface Boot;
    uses interface Leds;
    uses interface Timer<TMilli> as Timer;
    uses interface Sampling<sample_type>;
}
implementation 
{
    // ======================= Data ==================================
    enum { RATE_HZ = 2 };
    enum { NBR_ADC_CHANS = 3 };
    uint8_t ADC_CHANS[NBR_ADC_CHANS] = {3, 4, 5};  // {0, 1, 2};
    enum { NBR_BUFFS = 3};
    sample_type samplesBuff[NBR_BUFFS][NBR_ADC_CHANS];
    uint16_t currBuffIndex = 0;


    // ======================= Methods ===============================
    task void startSampling()
    {
        call Sampling.startSampling(ADC_CHANS, NBR_ADC_CHANS, RATE_HZ, &samplesBuff[0][0]);    
    }
    
    event void Boot.booted()
    {
        printfUART_init();
        call Timer.startPeriodic(5000);
        post startSampling();
    }

    event void Timer.fired()
    {
        static uint16_t cntFired = 0;
        cntFired++;
        call Leds.led0Toggle();
        printfUART("TestP::Timer.fired() - cntFired= %u\n", cntFired);
    }
    
    void task printSamples()
    {
        uint8_t b = 0;
        uint8_t c = 0;

        printfUART("samplesBuff[%u][%u] = {", NBR_BUFFS, NBR_ADC_CHANS);
        for (b = 0; b < NBR_BUFFS; ++b) {
            for (c = 0; c < NBR_ADC_CHANS; ++c) {
                printfUART(" %u", samplesBuff[b][c]);
            }
            printfUART(" ");
        }
        printfUART("}\n");
    }    

    async event sample_type* Sampling.dataReady(sample_type *destAddr, error_t returnResult)
    {
        call Leds.led1Toggle();
        post printSamples();
        currBuffIndex = (currBuffIndex + 1) % NBR_BUFFS;
        return &samplesBuff[currBuffIndex][0];
    }
}


