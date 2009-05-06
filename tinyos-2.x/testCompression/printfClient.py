#!/usr/bin/env python
import sys
import os
sys.path.append(os.path.join(os.environ["TOSROOT"], "support/sdk/python"))
from tinyos.message import MoteIF
import printfMsg

class MsgListener:
    def __init__(self):
        self.mote = MoteIF.MoteIF()
        self.mote.addSource("sf@localhost:9002")
        self.mote.addListener(self, printfMsg.printfMsg)
        
    def receive(self, src, msg):
        msgType = msg.get_amType()
        if msgType == printfMsg.AM_TYPE:
            sys.stdout.write(msg.getString_buffer())
                
if __name__ == "__main__":
    m = MsgListener()
