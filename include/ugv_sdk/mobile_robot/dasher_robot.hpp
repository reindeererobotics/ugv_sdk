/*
 * dasher_robot.hpp
 */

#ifndef DASHER_ROBOT_HPP
#define DASHER_ROBOT_HPP

//#include "lib/ugv_sdk/include/ugv_sdk/details/interface/robot_common_interface.hpp"
//#include "lib/ugv_sdk/include/ugv_sdk/details/interface/scout_interface.hpp"
#include "ugv_sdk/mobile_robot/scout_robot.hpp"

#include <memory>

#include "ugv_sdk/details/interface/robot_common_interface.hpp"
#include "ugv_sdk/details/interface/dasher_interface.hpp"

namespace dasher {
class DasherRobot : public westonrobot::ScoutRobot::ScoutRobot{
 public:
  DasherRobot();
  ~DasherRobot();

  bool Connect(std::string can_name) override;

  void EnableCommandedMode() override;

  void SetMotionCommand(double linear_vel, double angular_vel) override;
  void SetLightCommand(AgxLightMode f_mode, uint8_t f_value, AgxLightMode r_mode,
                       uint8_t r_value) override;
  void DisableLightControl() override;

  void ResetRobotState() override;

  ProtocolVersion GetParserProtocolVersion() override;

  // get robot state
  westonrobot::ScoutCoreState GetDasherRobotState();
  westonrobot::ScoutActuatorState GetDasherActuatorState();
  DasherCommonSensorState GetDasherCommonSensorState();

 protected:
  RobotCommonInterface* robot_;

};
}  // namespace dasher

#endif /* DASHER_ROBOT_HPP */
