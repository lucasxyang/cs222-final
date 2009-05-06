#include "Sampling.h"
#include "printf.h"
#include "sample.h"

generic module TestCompressionP(typedef sample_type)
{
  uses interface Boot;
  uses interface Leds;
  uses interface Sampling<sample_type>;
}

implementation 
{
  // ======================= Data ==================================
  enum { RATE_HZ = 100 };
  enum { NBR_ADC_CHANS = 1 };
  //uint8_t ADC_CHANS[NBR_ADC_CHANS] = {3, 4, 5};  // {0, 1, 2};
  uint8_t ADC_CHANS[NBR_ADC_CHANS] = {3};  // {0, 1, 2};
  enum { NBR_BUFFS = 3};
  sample_type samplesBuff[NBR_BUFFS][SAMPLE_MSG_LENGTH];
  uint16_t currBuffIndex = 0;
  uint16_t currSampleIndex = 0;
  norace uint16_t currPrintIndex = 0;
  
  
  // ======================= Methods ===============================
  task void startSampling()
  {
    call Sampling.startSampling(ADC_CHANS, NBR_ADC_CHANS, RATE_HZ, &samplesBuff[0][0]);    
  }
  
  event void Boot.booted()
  {
    post startSampling();
  }
  
  void task printSamples()
  {
    uint8_t i = 0;
    uint16_t pIndex;
    
    atomic {
      pIndex = currPrintIndex;
    }
    printf("samplesBuff[%u] = {", currPrintIndex);
    for (i = 0; i < SAMPLE_MSG_LENGTH; ++i) {
      printf(" %u", samplesBuff[currPrintIndex][i]);
    }
    printf(" }\n");
    printfflush();
  }	 
  
  async event sample_type* Sampling.dataReady(sample_type *destAddr, error_t returnResult)
  {
    currSampleIndex++;
    if (currSampleIndex >= SAMPLE_MSG_LENGTH) { 
      currSampleIndex = 0;
      currPrintIndex = currBuffIndex;
      call Leds.led1Toggle();
      post printSamples();
      currBuffIndex = (currBuffIndex + 1) % NBR_BUFFS;
    }
    return &samplesBuff[currBuffIndex][currSampleIndex];
  }
}
