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
#include "ugv_sdk/details/interface/dasher_interface.hpp"
#include "reindeere/reindeere_message.h"
#include "reindeere/dasher_msg_parser.h"
#include "agilex/interface/agilex_message.h"

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
    return ScoutBase::GetRobotState();
  }

  westonrobot::ScoutActuatorState GetActuatorState() override {
    return ScoutBase::GetActuatorState();
  }
  
  private:
  /*DO NOT USE*/
  westonrobot::ScoutCommonSensorState GetCommonSensorState() override {
    return ScoutBase::GetCommonSensorState();
  }

  public:
   DasherCommonSensorState GetSensorState() {
    DasherCommonSensorState state = ScoutToDasherSensorState(&GetCommonSensorState());

    

    return state;
  }

  /*converts a westonrobot::ScoutCommonSensorState` object to a `DasherCommonSensorState` object. 
  It takes a pointer to a `westonrobot::ScoutCommonSensorState` object as input and returns a `DasherCommonSensorState` object. 
  The function copies the relevant fields from the `westonrobot::ScoutCommonSensorState` object to the `DasherCommonSensorState` object and returns it.
  */
  DasherCommonSensorState ScoutToDasherSensorState(westonrobot::ScoutCommonSensorState* scout){
    DasherCommonSensorState state;
    state.bms_basic_state = scout->bms_basic_state;
    state.time_stamp = scout->time_stamp;

    return state;
  }


  bool Connect(std::string can_name) override {
    return ConnectPort(can_name,
                       std::bind(ParseCANFrame_wrapper, this,
                                 std::placeholders::_1));
  }

AgilexBase<ParserInterface<ProtocolVersion::AGX_V2>> base = new AgilexBase<ParserInterface<ProtocolVersion::AGX_V2>>();


class W_ParseCANFrame {
public:
  void ParseCANFrame(can_frame *rx_frame) {
    ReindeereMessage status_msg;

    if (DecodeDasherFrame(rx_frame, &status_msg)) {
      &AgilexBase<ParserInterface<ProtocolVersion::AGX_V2>>::UpdateRobotCoreState(ReinToAgxMsg(&status_msg));
      UpdateActuatorState(ReinToAgxMsg(&status_msg));
      UpdateCommonSensorState(status_msg);
      UpdateResponseVersion(ReinToAgxMsg(&status_msg));
      UpdateMotorState(ReinToAgxMsg(&status_msg));
    }
  }
};

W_ParseCANFrame* parseCANFrame;

 void ParseCANFrame_wrapper(can_frame *rx_frame)
{
  parseCANFrame->ParseCANFrame(rx_frame);
}


static void UpdateCommonSensorState(const ReindeereMessage &status_msg) {
    std::lock_guard<std::mutex> guard(common_sensor_state_mtx_);
    //    std::cout << common_sensor_state_msgs_.bms_basic_state.battery_soc<<
    //    std::endl;
    switch (status_msg.type) {
      case AgxMsgBmsBasic: {
        //      std::cout << "system status feedback received" << std::endl;
        common_sensor_state_msgs_.time_stamp = westonrobot::AgxMsgRefClock::now();
        common_sensor_state_msgs_.bms_basic_state =
            status_msg.body.bms_basic_msg;
        break;
      }
      case DasherMsgUltrasonic: {
        common_sensor_state_msgs_.time_stamp =westonrobot::AgxMsgRefClock::now();
        common_sensor_state_msgs_.us_state = status_msg.body.ultrasonic_msg;
      }
      default:
        break;
    }
  }

  


};



}  // namespace dasher

//#include "lib/ugv_sdk/include/ugv_sdk/details/protocol_v1/protocol_v1_parser.hpp"
//#include "lib/ugv_sdk/include/ugv_sdk/details/protocol_v2/protocol_v2_parser.hpp"

namespace dasher {

using DasherBaseV2 = DasherBase;

}  // namespace westonrobot

#endif /* DASHER_BASE_HPP */
