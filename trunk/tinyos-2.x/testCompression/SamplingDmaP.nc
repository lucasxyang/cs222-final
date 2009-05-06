/*
 * Copyright (c) 2008
 *	The President and Fellows of Harvard College.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
/**
 * <pre>URL: http://www.eecs.harvard.edu/~konrad/projects/shimmer</pre>
 * @author Konrad Lorincz
 * @version 1.0, November 10, 2006
 */

#include "Sampling.h"

generic module SamplingDmaP(typedef sample_type) 
{
  provides interface Sampling<sample_type>; 
  
  uses interface Boot;
  uses interface HplAdc12;
  
  uses interface Msp430DmaControl;
  uses interface Msp430DmaChannel;
  
#ifdef SAMPLING_USE_TIMERA
  uses interface Msp430Timer as TimerA;
  uses interface Msp430TimerControl as ControlA0;
  uses interface Msp430Compare as CompareA0;
  uses interface Leds;
#else
  uses interface Timer<TMilli>;
#endif
}
implementation 
{
  // ======================= Data ==================================
  bool isSampling = FALSE;
  uint8_t nbrSamplingChannels = 0;
  
  
  // ======================= Methods ===============================
  void resetADC12Registers(adcport_t adcPorts[], uint8_t nbrAdcPorts);
  void stopConversion();
  void resetTimerA();
  void startTimerA(uint8_t clockSrc, uint8_t clockDiv, uint16_t samplingPeriodJiffies);
  
  
  // ----------------------- DMA code -------------------------------
  async event void Msp430DmaChannel.transferDone(error_t success) 
  {   
    // DMA0DA: the destination addres register points to the destination address for single-transfers 
    // or the first address for block-transfers.  DMA0DA remains unchanged during clock and 
    // bust-block transfers.
    sample_type *nextDestAddr = signal Sampling.dataReady((sample_type*)DMA0DA, success);
    if (nextDestAddr == NULL)
      call Sampling.stopSampling();
    else
      call Msp430DmaChannel.repeatTransfer((void*)ADC12MEM0_, nextDestAddr, nbrSamplingChannels);
  }
  
  void setupDMA(sample_type *destAddr)
  {
    call Msp430DmaControl.setFlags(FALSE, TRUE, FALSE);  // sets DMACTL1
    
    call Msp430DmaChannel.setupTransfer(DMA_REPEATED_BLOCK_TRANSFER,  //dma_transfer_mode_t transfer_mode, 
					DMA_TRIGGER_ADC12IFGx,        //dma_trigger_t trigger, 
					DMA_EDGE_SENSITIVE,           //dma_level_t level,
					(void*)ADC12MEM0_,            //void *src_addr, 
					(void*)destAddr,              //void *dst_addr, 
					nbrSamplingChannels,          //uint16_t size,
					DMA_WORD,                     //dma_byte_t src_byte, 
					DMA_WORD,                     //dma_byte_t dst_byte,
					DMA_ADDRESS_INCREMENTED,      //dma_incr_t src_incr, 
					DMA_ADDRESS_INCREMENTED);     //dma_incr_t dst_incr
    
    call Msp430DmaChannel.startTransfer();
  }
  
  
  event void Boot.booted()
  {
    atomic isSampling = FALSE;
#ifdef SAMPLING_USE_TIMERA
    resetTimerA();
#endif
    
    // XXX This does not belong here!
#ifdef PLATFORM_SHIMMER
    // init the accel
    TOSH_MAKE_ACCEL_SLEEP_N_OUTPUT();         // sleep for accel
    TOSH_SEL_ACCEL_SLEEP_N_IOFUNC();
    
    TOSH_MAKE_ADC_ACCELZ_INPUT();
    TOSH_SEL_ADC_ACCELZ_MODFUNC();
    
    TOSH_MAKE_ADC_ACCELY_INPUT();
    TOSH_SEL_ADC_ACCELY_MODFUNC();
    
    TOSH_MAKE_ADC_ACCELX_INPUT();
    TOSH_SEL_ADC_ACCELX_MODFUNC();
    
    // set the sensitivity to 4G
    TOSH_CLR_ACCEL_SEL0_PIN();
    TOSH_SET_ACCEL_SEL1_PIN();
    
    // turn on accel on shimmer
    TOSH_SET_ACCEL_SLEEP_N_PIN();    // wakes up accel board
#endif
  }
  
  uint16_t hzToJiffies(uint16_t hz)
  {
#define ACLK_HZ 32768UL
    uint32_t res = ACLK_HZ/hz;
    if ((double)ACLK_HZ/(double)hz - (double)res >= 0.5) // round
      res++;
    
    if (res > 65536) {
      //exit(1);        // abort // KLDEBUG removed exit(1) because of compiler bug
      return res;  // this line will never be reached
    }
    else
      return res;
  }
  
  uint32_t hzToBinaryMillisec(uint16_t hz)
  {
    uint32_t ms = 1024/hz;
    if ((double)1024.0/(double)hz - (double)ms >= 0.5) // round
      ms++;
    
    return ms;
  }
  
  command error_t Sampling.startSampling(adcport_t adcPorts[], uint8_t nbrAdcPorts, uint16_t rateHz, sample_type *destAddr)
  {
    atomic {
      if (isSampling == TRUE || nbrAdcPorts > SAMPLING_MAX_ADCPORTS || rateHz < 1)
	return FAIL;
      else {
	nbrSamplingChannels = nbrAdcPorts;
	isSampling = TRUE;
      }
    }
    
    // (1) - Setup ADC12
    resetADC12Registers(adcPorts, nbrAdcPorts);
    
    // (2) - Setup DMA
    setupDMA(destAddr);
    
    // (3) - Setup Timer
#ifdef SAMPLING_USE_TIMERA
    startTimerA(MSP430TIMER_ACLK , MSP430TIMER_CLOCKDIV_1, hzToJiffies(rateHz));
#else
    call Timer.startPeriodic(hzToBinaryMillisec(rateHz));
#endif
    return SUCCESS;
  }
  
  command bool Sampling.isSampling() 
  {
    atomic return isSampling;
  }
  
  command error_t Sampling.stopSampling() 
  {
    atomic {
      if (isSampling == TRUE) {
	stopConversion();
	isSampling = FALSE;
	call Msp430DmaChannel.stopTransfer();
#ifdef SAMPLING_USE_TIMERA
	resetTimerA();
#else
	call Timer.stop();
#endif
	return SUCCESS;
      }
      else
	return SUCCESS;
    }            
  }
  
  
  // ----------------------- ADC12 driver code -------------------------------
  void configureAdcPin(uint8_t inputChannel)
  {
    if( inputChannel <= 7 ){
      P6SEL |= (1 << inputChannel);  //adc function (instead of general IO)
      P6DIR &= ~(1 << inputChannel); //input (instead of output)
    }
  }
  
  void initADC12CTL0()
  {
    adc12ctl0_t ctl0 = {
      adc12sc:0,                      // start conversion: 0 = no sample-and-conversion-start
      enc:0,                          // enable conversion: 0 = ADC12 disabled
      adc12tovie:0,                   // conversion-time-overflow-interrupt: 0 = interrupt dissabled
      adc12ovie:0,                    // ADC12MEMx overflow-interrupt: 0 = dissabled
      adc12on:1,                      // ADC12 on: 1 = on
      refon:0,                        // reference generator: 0 = off
      r2_5v:1,                        // reference generator voltage: 1 = 2.5V
      msc:1,                          // multiple sample and conversion: 1 = conversions performed ASAP
      sht0:SAMPLE_HOLD_4_CYCLES,    // sample-and-hold-time for  ADC12MEM0 to ADC12MEM7  
      sht1:SAMPLE_HOLD_4_CYCLES};   // sample-and-hold-time for  ADC12MEM8 to ADC12MEM15  
    
    call HplAdc12.setCtl0(ctl0);
  }
  
  void initADC12CTL1()
  {
    adc12ctl1_t ctl1 = {
      adc12busy:0,                    // no operation is active
      conseq:1,                       // conversion mode: sequence of chans
      adc12ssel:SHT_SOURCE_ADC12OSC,  // SHT_SOURCE_ADC12OSC=0; ADC12 clocl source
      adc12div:SHT_CLOCK_DIV_1,       // SHT_CLOCK_DIV_1=0; ADC12 clock div 1
      issh:0,                         // sample-input signal not inverted
      shp:1,                          // Sample-and-hold pulse-mode select: SAMPCON signal is sourced from the sampling timer
      shs:0,                          // Sample-and-hold source select= ADC12SC bit
      cstartadd:0};                   // conversion start addres ADC12MEM0
    
    call HplAdc12.setCtl1(ctl1);
  }
  
  void initADC12MEMCTLx(adcport_t adcPorts[], uint8_t nbrAdcPorts)
  {
    adc12memctl_t memctl = {
      inch: 0,                        // input channel: ADC0
      sref: REFERENCE_AVcc_AVss,      // reference voltage: 
      eos: 1 };                       // end of sequence flag: 1 indicates last conversion
    
    uint8_t i = 0;
    if (nbrAdcPorts > 16) {
      //exit(1); // KLDEBUG removed exit(1) because of compiler bug
    }
    
    for (i = 0; i < nbrAdcPorts; ++i) {
      memctl.inch = adcPorts[i];
      configureAdcPin(memctl.inch);
      if (i < nbrAdcPorts-1)
	memctl.eos = 0;
      else {
	memctl.eos = 1;                   // eos=1 indicates last conversion in sequence
	//call HplAdc12.setIEFlags(1 << i); // Set interupt for last register in sequence MUST DISABLE with DMA
      }
      call HplAdc12.setMCtl(i, memctl);
    }
  }
  
  void resetADC12Registers(adcport_t adcPorts[], uint8_t nbrAdcPorts)
  {
    initADC12CTL0();
    initADC12CTL1();
    initADC12MEMCTLx(adcPorts, nbrAdcPorts);
  }
  
  void stopConversion()
  {
    call HplAdc12.stopConversion();
    call HplAdc12.setIEFlags(0);
    call HplAdc12.resetIFGs();
  }                                            
  
  async event void HplAdc12.conversionDone(uint16_t iv)  {}
  
  // ----------------------- Timer code -------------------------------
#ifdef SAMPLING_USE_TIMERA
  void resetTimerA()
  {
    msp430_compare_control_t ccResetSHI = {
      ccifg : 0, cov : 0, out : 0, cci : 0, ccie : 0,
      outmod : 0, cap : 0, clld : 0, scs : 0, ccis : 0, cm : 0 };
    
    call TimerA.setMode(MSP430TIMER_STOP_MODE);
    call TimerA.clear();
    call TimerA.disableEvents();
    
    call ControlA0.setControl(ccResetSHI);
  }
  
  void startTimerA(uint8_t clockSrc, uint8_t clockDiv, uint16_t samplingPeriodJiffies)
  {
    // (1) - Set the sampling rate: clock, clockDivider, and samplingPeriodJiffies
    call TimerA.setClockSource(clockSrc);
    // NOTE: TimerA.setInputDivider() has a bug! So, I'm modifying the register directly.
    //call TimerA.setInputDivider(SAMPCON_CLOCK_DIV_2);
    atomic TACTL |= (clockDiv << 6);
    
    call CompareA0.setEvent(samplingPeriodJiffies-1);
    call ControlA0.enableEvents();
    
    // (2) - Start the timer
    call TimerA.setMode(MSP430TIMER_UP_MODE); // go!
  }
  
  async event void TimerA.overflow() {}
  
  async event void CompareA0.fired()
  {
    // Trigger a sequence of conversions
    call HplAdc12.startConversion();
    call Leds.led0Toggle();
  }
#else
  event void Timer.fired()
  {
    // Trigger a sequence of conversions
    call HplAdc12.startConversion();
  }
#endif
}

