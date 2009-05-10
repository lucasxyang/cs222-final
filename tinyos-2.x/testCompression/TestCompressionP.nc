#include "Sampling.h"
#include "printf.h"
#include "sample.h"

generic module TestCompressionP(typedef sample_type)
{
  uses interface Boot;
  uses interface Leds;
  uses interface Sampling<sample_type>;
  uses interface AMSend;
  uses interface SplitControl as AMControl;
  //uses interface Packet;
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
  bool sendBusy = FALSE; // are we currently sending a message
  message_t sendbuf;    // buffer for our send message
  
  // ======================= Methods ===============================
  task void startSampling()
  {
    call Sampling.startSampling(ADC_CHANS, NBR_ADC_CHANS, RATE_HZ, &samplesBuff[0][0]);    
  }
  
  event void Boot.booted()
  {
    call AMControl.start();
  }

  event void AMControl.startDone(error_t err) {
    if (err == SUCCESS) {
      post startSampling();  
    }
    else {
      call AMControl.start();
    }
  }

  event void AMControl.stopDone(error_t err) {
    // do nothing
  }

  event void AMSend.sendDone(message_t* msg, error_t error) {
    sendBusy = FALSE;
  }
  
  
  void task sendSamples()
  {
    sample_msg_t* msgPtr;
    uint16_t pIndex;
    atomic {
      pIndex = currPrintIndex;
    }

    if (sendBusy == TRUE) {
      post sendSamples();
      return;
    }
    else {
      // Send the message
      msgPtr = (sample_msg_t *)call AMSend.getPayload(&sendbuf, 
						      sizeof(sample_msg_t));

      memcpy(msgPtr, &samplesBuff[pIndex][0], sizeof(sample_msg_t));

      if (call AMSend.send(AM_BROADCAST_ADDR, &sendbuf, 
			   sizeof(sample_msg_t)) == SUCCESS)
        sendBusy = TRUE;
    }
  }
  
  void task printSamples()
  {
    uint8_t i = 0;
    uint16_t pIndex;
    
    atomic {
      pIndex = currPrintIndex;
    }
    printf("samplesBuff[%u] = {", pIndex);
    for (i = 0; i < SAMPLE_MSG_LENGTH; ++i) {
      printf(" %u", samplesBuff[pIndex][i]);
    }
    printf(" }\n");
    printfflush();
  }	 
  
  async event sample_type* Sampling.dataReady(sample_type *destAddr, 
					      error_t returnResult)
  {
    currSampleIndex++;
    if (currSampleIndex >= SAMPLE_MSG_LENGTH) { 
      currSampleIndex = 0;
      currPrintIndex = currBuffIndex;
      call Leds.led1Toggle();
      post sendSamples();
      //post printSamples();
      currBuffIndex = (currBuffIndex + 1) % NBR_BUFFS;
    }
    return &samplesBuff[currBuffIndex][currSampleIndex];
  }
}
