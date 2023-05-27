/*
 * scout_robot_demo.cpp
 *
 * Created on: Jul 08, 2021 11:12
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include <unistd.h>

#include <memory>
#include <iostream>

#include "ugv_sdk/mobile_robot/dasher_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

using namespace westonrobot;
using namespace dasher;

int main(int argc, char **argv) {
  std::string device_name="can0";
  std::string robot_subtype="dasher";

  // argv[1] = ;

  // if (argc == 2) {
  //   device_name = {argv[1]};
  //   std::cout << "Selected interface " << device_name << ", robot type: dasher"
  //             << std::endl;
  // } else if (argc == 3) {
  //   robot_subtype = {argv[1]};
  //   device_name = {argv[2]};
  //   std::cout << "Selected interface " << device_name
  //             << ", robot type: " << robot_subtype << std::endl;
  // } else {
  //   std::cout << "Usage: demo_dasher_robot [<robot-subtype>] <interface>"
  //             << std::endl
  //             << "Example 1: ./demo_dasher_robot can0" << std::endl
  //             << "\t <robot-subtype>: mini" << std::endl;
  //   return -1;
  // }

  // bool is_dasher_mini = false;
  // if (robot_subtype == "mini") {
  //   is_dasher_mini = true;
  // } else if (!robot_subtype.empty() && robot_subtype != "dasher") {
  //   std::cout
  //       << "Unkonwn robot subtype. Supported subtypes: \"dasher\" or \"mini\""
  //       << std::endl;
  // }

  std::unique_ptr<DasherRobot> dasher = std::unique_ptr<DasherRobot>( new DasherRobot());

  if (dasher == nullptr)
    std::cout << "Failed to create robot object" << std::endl;

  dasher->Connect(device_name);

  if (dasher->GetParserProtocolVersion() == ProtocolVersion::AGX_V2) {
    dasher->EnableCommandedMode();
  }

  // light control
  std::cout << "Light: const off" << std::endl;
  dasher->SetLightCommand(CONST_OFF, 0, CONST_OFF, 0);
  sleep(3);
  std::cout << "Light: const on" << std::endl;
  dasher->SetLightCommand(CONST_ON, 0, CONST_ON, 0);
  sleep(3);
  std::cout << "Light: breath" << std::endl;
  dasher->SetLightCommand(BREATH, 0, BREATH, 0);
  sleep(3);
  std::cout << "Light: custom 30-40" << std::endl;
  dasher->SetLightCommand(CUSTOM, 30, CUSTOM, 40);
  sleep(3);
  dasher->SetLightCommand(CONST_OFF, 0, CONST_OFF, 0);

  int count = 0;
  while (true) {
    // motion control
    std::cout << "Motor: 1.0, 0" << std::endl;
    dasher->SetMotionCommand(1.0, 0.0);

    // get robot state
    auto state = dasher->GetRobotState();
    std::cout << "-------------------------------" << std::endl;
    std::cout << "count: " << count << std::endl;
    std::cout << "control mode: "
              << static_cast<int>(state.system_state.control_mode)
              << " , vehicle state: "
              << static_cast<int>(state.system_state.vehicle_state)
              << " , error code: " << std::hex << state.system_state.error_code
              << std::dec
              << ", battery voltage: " << state.system_state.battery_voltage
              << std::endl;
    std::cout << "velocity (linear, angular): "
              << state.motion_state.linear_velocity << ", "
              << state.motion_state.angular_velocity << std::endl;
    std::cout << "core state age (ms): "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     AgxMsgRefClock::now() - state.time_stamp)
                     .count()
              << std::endl;

    auto actuator = dasher->GetActuatorState();
    if (dasher->GetParserProtocolVersion() == ProtocolVersion::AGX_V1) {
      for (int i = 0; i < 4; ++i) {
        printf("motor %d: current %f, rpm %d, driver temp %f, motor temp %f\n",
               actuator.actuator_state[i].motor_id,
               actuator.actuator_state[i].current,
               actuator.actuator_state[i].rpm,
               actuator.actuator_state[i].driver_temp,
               actuator.actuator_state[i].motor_temp);
      }
      std::cout << "actuator state age (ms): "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                       AgxMsgRefClock::now() - actuator.time_stamp)
                       .count()
                << std::endl;
    } else {
      for (int i = 0; i < 4; ++i) {
        printf("motor %d: current %f, rpm %d, driver temp %f, motor temp %f\n",
               actuator.actuator_hs_state[i].motor_id,
               actuator.actuator_hs_state[i].current,
               actuator.actuator_hs_state[i].rpm,
               actuator.actuator_ls_state[i].driver_temp,
               actuator.actuator_ls_state[i].motor_temp);
      }
      std::cout << "actuator state age (ms): "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                       AgxMsgRefClock::now() - actuator.time_stamp)
                       .count()
                << std::endl;
    }
    std::cout << "-------------------------------" << std::endl;

    usleep(20000);
    ++count;
  }

  return 0;
}