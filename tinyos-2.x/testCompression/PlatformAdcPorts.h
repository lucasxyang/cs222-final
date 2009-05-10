/*
 * Copyright (c) 2008
 *  The President and Fellows of Harvard College.
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
#ifndef PLATFORMADCPORTS_H
#define PLATFORMADCPORTS_H

typedef uint8_t adcport_t;

// Implementation supports up to 16 channels, due to 
// (1) - limit of uint16_t to hold 16 bits of channel information.
// (2) - there are 16 registers on msp430
enum {SAMPLING_MAX_ADCPORTS = 16};

enum {ADCPORT_INVALID = 255};

// The ports are platform specific
//#ifdef PLATFORM_SHIMMER
enum {
    ADCPORT_ACCX = 5,
    ADCPORT_ACCY = 4,
    ADCPORT_ACCZ = 3,
    ADCPORT_GYROX = 1,
    ADCPORT_GYROY = 6,
    ADCPORT_GYROZ = 2,
};
//#else
//    #error "*** Unknown platform in PlatformAdcPorts.h"
//#endif


inline void AdcPorts_init(adcport_t adcPorts[], uint8_t adcPortsSize) 
{
    memset(adcPorts, ADCPORT_INVALID, adcPortsSize);
}      


#endif
 
 
