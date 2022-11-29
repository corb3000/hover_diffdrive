#ifndef HOVER_COMMS_H
#define HOVER_COMMS_H


#include <cstring>
#include <string>
#include <libserial/SerialPort.h>
#include "protocol.h"
#include "config.h"

// using namespace LibSerial ;

class HoverComms
{
 
public:
 
  void setup();
  SerialFeedback readValues();
  void setMotorValues(double joints [2]);
  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);

  LibSerial::SerialPort  serial_conn; //< Underlying serial connection

  SerialFeedback read_msg;
  SerialCommand write_msg;
 
  HoverComms();

private:
  
  void checkDecode();
  void encoder_update (int16_t right, int16_t left);
  static const uint8_t read_lenth = 22;
  char read_buffer [read_lenth] = { };

};

#endif // HOVER_COMMS_H