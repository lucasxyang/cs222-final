#!/usr/bin/env python
import sys, os
import struct

SOURCE_PATH = "../data/shimmer/"
SOURCE_FILE = SOURCE_PATH + "shimmer.data"

def getData(file_path):
    data = {'accx':[],'accy':[],'accz':[],'gyro1':[],'gyro2':[],'gyro3':[]}
    file = open(file_path, 'r')
    size = 0
    for line in file:
        size = size + 1
        entries = line.split(' ')
        data['accx'].append(entries[1])
        data['accy'].append(entries[2])
        data['accz'].append(entries[3])
        data['gyro1'].append(entries[4])
        data['gyro2'].append(entries[5])
        data['gyro3'].append(entries[6])

    return data, size

def packData(data, size):
    accx = open(SOURCE_PATH+"accx", 'w')
    accy = open(SOURCE_PATH+"accy", 'w')
    accz = open(SOURCE_PATH+"accz", 'w')
    gyro1 = open(SOURCE_PATH+"gyro1", 'w')
    gyro2 = open(SOURCE_PATH+"gyro2", 'w')
    gyro3 = open(SOURCE_PATH+"gyro3", 'w')
    full = open(SOURCE_PATH+"full", 'w')

    for i in range(0, size):
        accx.write(struct.pack("<H", int(data['accx'][i])))
        accy.write(struct.pack("<H", int(data['accy'][i])))
        accz.write(struct.pack("<H", int(data['accz'][i])))
        gyro1.write(struct.pack("<H", int(data['gyro1'][i])))
        gyro2.write(struct.pack("<H", int(data['gyro2'][i])))
        gyro3.write(struct.pack("<H", int(data['gyro3'][i])))

        full.write(struct.pack("<HHHHHH", int(data['accx'][i]),\
                                   int(data['accy'][i]), \
                                   int(data['accz'][i]), \
                                   int(data['gyro1'][i]), \
                                   int(data['gyro2'][i]), \
                                   int(data['gyro3'][i])))
                   
                   

data, size = getData(SOURCE_FILE)
packData(data, size)
    
