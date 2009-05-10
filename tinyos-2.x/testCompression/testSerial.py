#!/usr/bin/env python

import sys,os
import time
sys.path.append(os.path.join(os.environ["TOSROOT"], "support/sdk/python"))
from tinyos.message import MoteIF
import SampleMsg

class MsgListener:
  def __init__(self):
    self.mif = MoteIF.MoteIF()
    self.source = self.mif.addSource("sf@localhost:9002")
    self.mif.addListener(self, SampleMsg.SampleMsg)

  def receive(self, src, msg):
    if msg.get_amType() == SampleMsg.AM_TYPE:
      data = msg.get_buffer()
      #print data
      for i in range(0,30,2):
        print int(data[i]) + (int(data[i+1])<<8)
      print ""

  def sendMsg(self, addr, amType, amGroup, msg):
    self.mif.sendMsg(self.source, addr, amType, amGroup, msg)

listener = MsgListener()

while True:
  time.sleep(1)
