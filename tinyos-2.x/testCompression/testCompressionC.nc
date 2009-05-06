#include "printf.h"

module testCompressionC {
  uses {
    // Interfaces for initialization:
    interface Boot;
    
    // Interfaces for communication
    interface Send;
    interface Intercept;
    interface Receive;
    interface CollectionPacket;
    interface RootControl;

    // Serial Communicaton
    interface AMSend as SerialSend;
    interface SplitControl as SerialControl;

    // Miscalleny:
    interface Timer<TMilli>;
    interface Leds;
    interface Read<uint16_t> as Sensor;
  }
}

implementation {
  static void startTimer();
  static void fatal_problem();
  static void report_problem();
  static void report_sent();
  static void report_received();

  message_t sendbuf;     // radio buffer for our message
  bool sendbusy = FALSE; // are we currently sending a message

  message_t serialsendbuf;     // serial buffer for our message
  bool serialsendbusy = FALSE; // are we currently sending a serial message

  uint32_t seqnum = 1;

  event void Boot.booted() {
    // Beginning our initialization phases:
    if (call RadioControl.start() != SUCCESS)
      fatal_problem();

    if (call SerialControl.start() != SUCCESS)
      fatal_problem();
    
    if (call RoutingControl.start() != SUCCESS)
      fatal_problem();
  }

  event void SerialControl.startDone(error_t error) {
    if (error != SUCCESS)
      fatal_problem();
  }

  event void RadioControl.startDone(error_t error) {
    if (error != SUCCESS)
      fatal_problem();

    // This is how to set yourself as a root to the collection layer:
    if (TOS_NODE_ID == 90) {
      call RootControl.setRoot();
    } 
    else {
      startTimer();
    }
  }

  static void startTimer() {
    if (call Timer.isRunning()) call Timer.stop();
    call Timer.startPeriodic(10*1024);
  }

  event void RadioControl.stopDone(error_t error) { }

  event void SerialControl.stopDone(error_t error) { }

  //
  // Only the root will receive messages from this interface; its job
  // is to forward them to the serial uart for processing on the pc
  // connected to the sensor network.
  //
  event message_t*
  Receive.receive(message_t* msg, void *payload, uint8_t len) {
    MultihopMsg* msgPtr;

    // Send the message out on the serial port
    if (!serialsendbusy) {
      msgPtr = (MultihopMsg *)call SerialSend.getPayload(&serialsendbuf, sizeof(MultihopMsg));
      if (msgPtr == NULL) {
        fatal_problem();
        return msg;
      }

      memcpy(msgPtr, payload, sizeof(MultihopMsg));

      if (call SerialSend.send(0xffff, &serialsendbuf, sizeof(MultihopMsg)) == SUCCESS)
        serialsendbusy = TRUE;
      else {
        report_problem();
      }
    }
    
    //msgPtr = (MultihopMsg*)payload;
    //printf("Basestation packet: source: %d, seqnum: %d, treedepth: %d, data: %d\n", 
    //       (int)msgPtr->source, (int)msgPtr->seqnum, 
    //       (int)msgPtr->treedepth, (int)msgPtr->data);
    //printfflush();
    
    return msg;
  }

  event void SerialSend.sendDone(message_t* msg, error_t error) {
    if (error == SUCCESS)
      report_sent();
    else {
      report_problem();
    }
    serialsendbusy = FALSE;
  }

  event bool Intercept.forward(message_t* msg, void* payload, uint8_t len){
    MultihopMsg *msgPtr;
    
    // Account for the hop in the message
    msgPtr = (MultihopMsg *)payload;
    msgPtr->treedepth = msgPtr->treedepth + 1;

    // Request bandwidth
    // if granted
    return TRUE;
    // else return FALSE
  }

  event void Timer.fired() {
    // Read our sensor 
    if (call Sensor.read() != SUCCESS)
      fatal_problem();
  }

  event void Sensor.readDone(error_t error, uint16_t data) {
    MultihopMsg *msgPtr;
    if (!sendbusy) {
      msgPtr = (MultihopMsg *)call Send.getPayload(&sendbuf, sizeof(MultihopMsg));
      if (msgPtr == NULL) {
        fatal_problem();
        return;
      }
      msgPtr->source = TOS_NODE_ID;
      msgPtr->seqnum = seqnum++;
      msgPtr->treedepth = 1;
      msgPtr->data = data;
      
      if (call Send.send(&sendbuf, sizeof(MultihopMsg)) == SUCCESS)
        sendbusy = TRUE;
      else {
        report_problem();
      }
    }
  }



  event void Send.sendDone(message_t* msg, error_t error) {
    if (error == SUCCESS)
      report_sent();
    else {
      report_problem();
    }
    sendbusy = FALSE;
  }

  // Use LEDs to report various status issues.
  static void fatal_problem() { 
    call Leds.led0On(); 
    call Leds.led1On();
    call Leds.led2On();
    call Timer.stop();
  }

  static void report_problem() { call Leds.led0Toggle(); }  // Red 
  static void report_sent() { call Leds.led1Toggle(); }     // Green
  static void report_received() { call Leds.led2Toggle(); } // Blue
}
