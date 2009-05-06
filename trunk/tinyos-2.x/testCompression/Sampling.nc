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
#include "Sampling.h"

interface Sampling<sample_type> 
{
    /**
     * Starts sampling the specified channels at the given rate.
     * @param adcPorts[]    adc ports to sample
     * @param numAdcPorts   number of adc ports
     * @param rateHz        sampling rate in Hz
     * @param destAddr      where to write the samples
     * @return <code>SUCCESS</code> if it's not sampling and the channels and rate are valid;
     *         <code>FAIL</code> if it's currently sampling or the channels or rate is invalid.
     */
    command error_t startSampling(adcport_t adcPorts[], uint8_t numAdcPorts, uint16_t rateHz, sample_type *destAddr);
    
    /**
     * Returns true if it's currently sampling.
     * @return <code>TRUE</code> if it's sampling; <code>FALSE</code> otherwise.
     */
    command bool isSampling();
    
    /**
     * Stops all channels currently being sampled.
     * @return <code>SUCCESS</code> if sampling stopped succesfully on all channels;
     *         <code>FAIL</code> if at least one channel failed to stop.
     */
    command error_t stopSampling();

    /**
     * Signales when it sampled each of the specified channels.
     * @param destAddr where the last converted samples where written
     * @param result  SUCCESS if the sampling was successful; otherwise, an 
     *                error code indicating the error.
     * @return        pointer to next destination address
     */
    async event sample_type* dataReady(sample_type *destAddr, error_t returnResult);
}

