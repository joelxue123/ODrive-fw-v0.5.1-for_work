#ifndef __CAN_ENCOS_HPP_
#define __CAN_ENCOS_HPP_

#include "interface_can.hpp"

class CANEncos {
public:
   static void handle_can_message(can_Message_t& msg);



private:
};

#endif