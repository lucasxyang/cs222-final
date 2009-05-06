#!/usr/bin/env python

import sys,os
import time
sys.path.append(os.path.join(os.environ["TOSROOT"], "support/sdk/python"))
from tinyos.message import MoteIF
import printfMsg

class MsgListener:
  def __init__(self):
    self.mif = MoteIF.MoteIF()
    self.source = self.mif.addSource("sf@localhost:9002")
    self.mif.addListener(self, printfMsg.printfMsg)

  def receive(self, src, msg):
    if msg.get_amType() == printfMsg.AM_TYPE:
        sys.stdout.write(msg.getString_buffer())

  def sendMsg(self, addr, amType, amGroup, msg):
    self.mif.sendMsg(self.source, addr, amType, amGroup, msg)

listener = MsgListener()
msg = RadioCountMsg.RadioCountMsg()
count = 0

while True:
  count += 1
  print "send count %u" % (count)
  msg.set_counter(count)
  listener.sendMsg(0, RadioCountMsg.AM_TYPE, 0x22, msg)
  time.sleep(1)
