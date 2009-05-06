configuration TestCompressionC 
{
}
implementation 
{
    components MainC, new TestCompressionP(uint16_t), LedsC;

    TestCompressionP.Boot -> MainC;
    TestCompressionP.Leds -> LedsC;

    components new SamplingC(uint16_t);
    TestCompressionP.Sampling -> SamplingC;
}



