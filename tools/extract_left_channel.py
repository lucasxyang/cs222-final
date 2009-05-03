#!/usr/bin/env python
import wave
import sys
import struct

# Extract the left channel from a 16 bit stereo wave file and write it
# out as a mono wave file.  The script takes a filename as inmput and
# will write an outout wave file with the same name and ending with
# -mono.wav.  This is need as the origional marmot wave files are
# stereo with only the left channel containing marmot data.

# usage: extract_left_channel.py filename.wav

filename = sys.argv[1]

# open our file for reading
rf = wave.open(filename)

# open our file for writing
wf = wave.open(filename.split('.')[0]+'-mono'+'.wav','w')

if rf.getnchannels() != 2:
    print "Input wave file is not stereo.  Exiting."
    sys.exit()

if rf.getsampwidth() != 2:
    print "Input wave file is not 16bit.  Exiting."
    sys.exit()

wf.setparams(rf.getparams())
wf.setnchannels(1)

for frame in range(0,rf.getnframes()):
    data = rf.readframes(1)
    #print struct.unpack('hh', data)
    wf.writeframes(data[0]+data[1])

# all done
rf.close()
wf.close()
