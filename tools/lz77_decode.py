#!/usr/bin/env python
import sys, os

DICT_SIZE = 256

def readFile(file_path):
    data = []
    file = open(file_path, "r")
    for line in file:
        data.append(int(line.strip()))
    return data

def decodeChunk(chunk):
    dict = []
    decoded_data = []

    done = False
    index = 0
    while (not done):
        element = chunk[index]
        if (element == 0):
            data = chunk[index + 1]
            decoded_data.append(data)
            dict.append(data)
            index = index + 2
        elif (element == 1):
            length = chunk[index + 1]
            start_pos = DICT_SIZE - chunk[index + 2]
            print "length: " + str(length) + ", start_pos: " + str(start_pos)
            print dict
            
            if (length > start_pos):
                data = dict[(start_pos - start_pos):start_pos]
                data.append(dict[0])
            else:
                data = dict[(start_pos - length):start_pos]
            
            dict.extend(data)
            decoded_data.extend(data)
            index = index + 3
        else:
            done = True

    return decoded_data
        

if (len(sys.argv) < 2):
    print "Must specify file name\n"
    quit()
file_path = sys.argv[1]
data = readFile(file_path)
decoded_data = decodeChunk(data)
print(decoded_data)
    
