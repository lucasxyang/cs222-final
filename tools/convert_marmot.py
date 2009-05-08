#!/usr/bin/env python
import wave
import sys
import struct
import os

# Convert a wave file to a header file with an array of audio samples
# usage: make-signal-header.py filename.wav
#
# The script will write a header file filename.h

filename = sys.argv[1]

# open our file for reading
rf = wave.open(filename)

# for text
#outfile = open(os.path.basename(filename).split('.')[0]+'.txt','w')

# for binary
outfile = open(os.path.basename(filename).split('.')[0]+'.bin','w')

if rf.getnchannels() != 1:
    print "Input wave file is not mono.  Exiting."
    sys.exit()

if rf.getsampwidth() != 2:
    print "Input wave file is not 16bit.  Exiting."
    sys.exit()

signame = filename.split('.')[0].lower()
hname = signame.upper()
ndata = rf.getnframes()

# for text
#for frame in range(0,rf.getnframes()):
#    data = rf.readframes(1)
#    sample = int(struct.unpack('h', data)[0])
#    print >> outfile, "  %d," % (sample)

# for binary
for frame in range(0,rf.getnframes()):
    data = rf.readframes(1)
    outfile.write(data)

rf.close()
