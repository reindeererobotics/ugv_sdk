/*
 * dasher_robot.cpp
 *
 */

/*
#include "dasher_robot.hpp"
#include "dasher_base.hpp"

#include "ugv_sdk/details/interface/robot_common_interface.hpp"
#include "dasher_interface.hpp"
*/

#include "ugv_sdk/mobile_robot/dasher_robot.hpp"
#include "ugv_sdk/details/robot_base/dasher_base.hpp"

namespace dasher {
DasherRobot::DasherRobot() : westonrobot::ScoutRobot::ScoutRobot(){
      robot_ = new DasherBase();
}

DasherRobot::~DasherRobot() {
  if (robot_) delete robot_;
}

void DasherRobot::EnableCommandedMode() { robot_->EnableCommandedMode(); }

bool DasherRobot::Connect(std::string can_name) {
  return robot_->Connect(can_name);
}

void DasherRobot::Connect(std::string uart_name, uint32_t baudrate) {
  //   robot_->Connect(uart_name, baudrate);
}

void DasherRobot::ResetRobotState() { robot_->ResetRobotState(); }

ProtocolVersion DasherRobot::GetParserProtocolVersion() {
  return robot_->GetParserProtocolVersion();
}

void DasherRobot::SetMotionCommand(double linear_vel, double angular_vel) {
  auto dasher = dynamic_cast<DasherInterface*>(robot_);
  dasher->SetMotionCommand(linear_vel, angular_vel);
}

void DasherRobot::DisableLightControl() { robot_->DisableLightControl(); }

void DasherRobot::SetLightCommand(AgxLightMode f_mode, uint8_t f_value,
                                 AgxLightMode r_mode, uint8_t r_value) {
  auto dasher = dynamic_cast<DasherInterface*>(robot_);
  dasher->SetLightCommand(f_mode, f_value, r_mode, r_value);
}

DasherCoreState DasherRobot::GetDasherRobotState() {
  auto dasher = dynamic_cast<DasherInterface*>(robot_);
  return dasher->GetRobotState();
}

DasherActuatorState DasherRobot::GetDasherActuatorState() {
  auto dasher = dynamic_cast<DasherInterface*>(robot_);
  return dasher->GetActuatorState();
}

///////////////////////////////////////////////////////////////////////////

}  // namespace dasher