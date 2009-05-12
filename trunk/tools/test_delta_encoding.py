#!/usr/bin/env python
import sys, os
import struct
import math

BLOCK_SIZE = 64
DATA_SIZE_BITS = 12

def readFile(file_name):
    full_data = []
    file = open(file_name, "rb")
    while True:
        binary = file.read(2)
        if not binary: break
        data = struct.unpack("<H", binary)
        full_data.append(data[0])

    return full_data

def calculateBlockSize(block):
    max_difference = 0
    for i in range(0, len(block)):
        if i != 0:
            difference = block[i] - block[i-1]
            if abs(difference) > abs(max_difference):
                max_difference = difference
                
    #size = DATA_SIZE_BITS * (BLOCK_SIZE - 1)
    #Add the size of the deltas
    if (max_difference == 0):
        size = 0
    else:  
        size = math.ceil(math.log(abs(max_difference), 2)) * len(block)
    #Add the size of the first element
    size = size + DATA_SIZE_BITS
    #Add the size of the size value in the header
    size = size + math.log(DATA_SIZE_BITS, 2)
    return size
    
def calculateCompressedSize(data, block_size):
    size = 0
    num_blocks = len(data) / block_size
    for block_index in range(0, num_blocks):
        start = block_index*block_size
        end = start + block_size - 1
        block = data[start:end]
        size = size + calculateBlockSize(block)

    return size

def iterate(data):
    for i in range(1, 12):
        block_size = int(math.pow(2, i))
        print "Block Size: " + str(block_size)
        size = calculateCompressedSize(data, block_size)

        orig_size = len(data) * DATA_SIZE_BITS

        print "Compression ratio:" + str(float(size)/orig_size * 100)


if (len(sys.argv) < 2):
    print "Must specify input"
    quit()

data = readFile(sys.argv[1])
print data
iterate(data)

