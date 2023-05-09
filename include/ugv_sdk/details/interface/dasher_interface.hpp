/*
 * Dasher_interface.hpp
 */

#ifndef DASHER_INTERFACE_HPP
#define DASHER_INTERFACE_HPP


//#include "lib/ugv_sdk/include/ugv_sdk/details/interface/agilex_message.h"
//#include "lib/ugv_sdk/include/ugv_sdk/details/interface/robot_common_interface.hpp"

#include "ugv_sdk/details/interface/scout_interface.hpp"

namespace dasher {
class DasherCoreState : public westonrobot::ScoutCoreState {
  public:
   DasherCoreState(const westonrobot::ScoutCoreState& scout_state)
      : ScoutCoreState(scout_state) {}
};


class DasherActuatorState : public westonrobot::ScoutActuatorState {
  public:
   DasherActuatorState(const westonrobot::ScoutActuatorState& scout_state)
      : ScoutActuatorState(scout_state) {}
};


class DasherInterface : public westonrobot::ScoutInterface {
  public:
  virtual ~DasherInterface() = default;

//   virtual void Connect(std::string uart_name, uint32_t baudrate){
//       // use derived version
//   };

//   virtual void SetMotionCommand(double linear_vel, double angular_vel) = 0;
//   virtual void SetLightCommand(AgxLightMode f_mode, uint8_t f_value,
//                                AgxLightMode r_mode, uint8_t r_value) = 0;

  // get robot state

   virtual westonrobot::ScoutCoreState GetRobotState() = 0;
   virtual westonrobot::ScoutActuatorState GetActuatorState() = 0;
};




}

#endif /* DASHER_INTERFACE_HPP */