#include "sample.h"

configuration TestCompressionC 
{
}
implementation 
{
  components MainC, new TestCompressionP(uint16_t), LedsC;
  components new SamplingC(uint16_t);

  TestCompressionP.Sampling -> SamplingC;
  TestCompressionP.Boot -> MainC;
  TestCompressionP.Leds -> LedsC;

  // Serial Communication
  //components SerialActiveMessageC;
  //components new SerialAMSenderC(AM_SAMPLE_MSG);
  //TestCompressionP.AMSend -> SerialAMSenderC;
  //TestCompressionP.AMControl -> SerialActiveMessageC;
  
  // Radio Communication
  components ActiveMessageC;
  components new AMSenderC(AM_SAMPLE_MSG);
  TestCompressionP.AMSend -> AMSenderC;
  TestCompressionP.AMControl -> ActiveMessageC;

  //TestCompressionP.Packet -> AMSenderC;
  
}
