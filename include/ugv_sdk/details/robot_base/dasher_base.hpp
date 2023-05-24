/*
 * dasher_base.hpp
 *
 * Created on: Dec 23, 2020 14:39
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef DASHER_BASE_HPP
#define DASHER_BASE_HPP

//#include "lib/ugv_sdk/include/ugv_sdk/details/interface/scout_interface.hpp"
//#include "lib/ugv_sdk/include/ugv_sdk/details/robot_base/agilex_base.hpp"
#include "ugv_sdk/details/robot_base/scout_base.hpp"

namespace dasher {
class DasherBase : public westonrobot::ScoutBase<westonrobot::ProtocolV2Parser> , public DasherInterface{
 public:
  DasherBase() : westonrobot::ScoutBase<westonrobot::ProtocolV2Parser>(){};
  virtual ~DasherBase() = default;

  bool Connect(std::string can_name) override {
    return westonrobot::ScoutBase<westonrobot::ProtocolV2Parser>::Connect(can_name);
  }

  void Connect(std::string uart_name, uint32_t baudrate) override {
    // TODO
  }


 // robot control

  void SetMotionCommand(double linear_vel, double angular_vel) override{
      westonrobot::ScoutBase<westonrobot::ProtocolV2Parser>::SendMotionCommand(
        linear_vel, angular_vel, 0.0, 0.0);
  }

  void SetLightCommand( AgxLightMode f_mode, uint8_t f_value,
                                AgxLightMode r_mode = AgxLightMode::CONST_ON,
                                uint8_t r_value = 0) override{
    westonrobot::ScoutBase<westonrobot::ProtocolV2Parser>::SendLightCommand(
        f_mode, f_value, r_mode, r_value);
  }



  westonrobot::ScoutCoreState GetRobotState() override {
    return DasherCoreState{ScoutBase::GetRobotState()};
  }

  westonrobot::ScoutActuatorState GetActuatorState() override {
    return DasherActuatorState{ScoutBase::GetActuatorState()};
  }

  westonrobot::ScoutCommonSensorState GetCommonSensorState() override{
    
    return DasherCommonSensorState{ScoutBase::GetCommonSensorState()};
  }

};


}  // namespace dasher

//#include "lib/ugv_sdk/include/ugv_sdk/details/protocol_v1/protocol_v1_parser.hpp"
//#include "lib/ugv_sdk/include/ugv_sdk/details/protocol_v2/protocol_v2_parser.hpp"

namespace dasher {

using DasherBaseV2 = DasherBase;

}  // namespace westonrobot

#endif /* DASHER_BASE_HPP */
