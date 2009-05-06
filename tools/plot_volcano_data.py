#!/usr/bin/env python
import sys, os
import matplotlib.pyplot as plt

DATA_DIR = "../data/volcano/"
FILE = DATA_DIR + "2005-08-11_03.18.30/node-200.dat"

def getFiles(top_dir):
    dirs = os.listdir(top_dir)
    tree = {}
    for dir in dirs:
        files = os.listdir(top_dir + dir)
        tree[dir] = files
    return tree

def getData(file):
    data_list = []
    f = open(file, 'r')
    for line in f:
        if (line[0] != '#'):
            line = line.strip()
            data_list.append(line)
            
    return data_list

def plotFile(file):
    list = getData(file)
    print min(list)
    print max(list)
    plt.figure()
    plt.plot(list)
    plt.title(file)
    plt.show()

def plotAllFiles(files):
    for dir, nodes in files.iteritems():
        for node in nodes:
            plotFile(DATA_DIR + "/" + dir + "/" + node)


#files = getFiles(DATA_DIR)
#plotAllFiles(files)



