/*
 * Copyright (c) 2006
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
 * @author Konrad Lorincz
 * @version 1.0, November 10, 2006
 */

// FIXME: Make this the default and move definition from here
#define SAMPLING_USE_TIMERA

generic configuration SamplingC(typedef sample_type) 
{
    provides interface Sampling<sample_type>;
} 
implementation 
{
#ifdef SAMPLING_FAKE
    // --------------- Fake version -----------------------
    components new SamplingFakeP(sample_type) as SamplingP;
#else 
  #ifdef SAMPLING_DMA
    // --------------- DMA version -----------------------
    components new SamplingDmaP(sample_type) as SamplingP;
    components Msp430DmaC;
    SamplingP.Msp430DmaControl -> Msp430DmaC;
    SamplingP.Msp430DmaChannel -> Msp430DmaC.Channel0; 
  #else 
    // --------------- Normal (Non-DMA) version -----------------------
    components new SamplingNoDmaP(sample_type) as SamplingP;
  #endif

    // --- Common wiring for DMA and Non-DMA ---
    components MainC, HplAdc12P;
    SamplingP.Boot -> MainC;
    SamplingP.HplAdc12 -> HplAdc12P;
#endif // SAMPLING_FAKE

    // --- Common wiring for Fake, DMA, and Non-DMA ---
    Sampling = SamplingP;


    // --- Timing Driver ---
#if defined(SAMPLING_FAKE) || !defined(SAMPLING_USE_TIMERA)
    components new TimerMilliC() as Timer;
    SamplingP.Timer -> Timer;
#else
    components Msp430TimerC;
    components NoLedsC as LedsC;
    SamplingP.TimerA -> Msp430TimerC.TimerA;
    SamplingP.ControlA0 -> Msp430TimerC.ControlA0;
    SamplingP.CompareA0 -> Msp430TimerC.CompareA0;
    SamplingP.Leds -> LedsC;
#endif
}


