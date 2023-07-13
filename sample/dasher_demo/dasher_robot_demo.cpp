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

int main(int argc, char **argv)
{

  std::unique_ptr<DasherRobot> dasher = std::unique_ptr<DasherRobot>(new DasherRobot());

  if (dasher == nullptr)
    std::cout << "Failed to create robot object" << std::endl;

  dasher->Connect("can0");

  if (dasher->GetParserProtocolVersion() == ProtocolVersion::AGX_V2)
  {
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
  while (true)
  {
    // motion control
    std::cout << "Motor: 1.0, 0" << std::endl;
    dasher->SetMotionCommand(1.0, 0.0);

    // get robot state
    auto state = dasher->GetDasherRobotState();
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

    auto actuator = dasher->GetDasherActuatorState();
    if (dasher->GetParserProtocolVersion() == ProtocolVersion::AGX_V1)
    {
      for (int i = 0; i < 4; ++i)
      {
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
    }
    else
    {
      for (int i = 0; i < 4; ++i)
      {
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

    auto sensor_state = dasher->GetDasherCommonSensorState();

    printf("bms_basic_state.battery_soc %d: bms_basic_state.battery_soh %d, bms_basic_state.temperature %f, bms_basic_state.voltage %f, bms_basic_state.current %f, us_state.distance %d, us_state.sensor_id %d \n",
           sensor_state.bms_basic_state.battery_soc,
           sensor_state.bms_basic_state.battery_soh,
           sensor_state.bms_basic_state.temperature,
           sensor_state.bms_basic_state.voltage,
           sensor_state.bms_basic_state.current,
           sensor_state.us_state.distance[0],
           sensor_state.us_state.sensor_id);

    printf("imu_accel_state{x,y,z}: %f,%f,%f",
                                              sensor_state.imu_accel_state.accel_x,
                                              sensor_state.imu_accel_state.accel_y,
                                              sensor_state.imu_accel_state.accel_z);
    
    printf("imu_gyro_state{x,y,z}: %f,%f,%f",
                                              sensor_state.imu_gyro_state.gyro_x,
                                              sensor_state.imu_gyro_state.gyro_y,
                                              sensor_state.imu_gyro_state.gyro_z);
    printf("imu_mag_state{x,y,z}: %f,%f,%f",
                                              sensor_state.imu_mag_state.mag_x,
                                              sensor_state.imu_mag_state.mag_y,
                                              sensor_state.imu_mag_state.mag_z);
    printf("imu_grav_state{x,y,z}: %f,%f,%f",
                                              sensor_state.imu_grav_state.grav_x,
                                              sensor_state.imu_grav_state.grav_y,
                                              sensor_state.imu_grav_state.grav_z);
    printf("imu_quat_state{w,x,y,z}: %f,%f,%f,%f",
                                              sensor_state.imu_quat_state.quat_x,
                                              sensor_state.imu_quat_state.quat_x,
                                              sensor_state.imu_quat_state.quat_y,
                                              sensor_state.imu_quat_state.quat_z);

    std::cout << "-------------------------------" << std::endl;

    usleep(20000);
    ++count;
  }

  return 0;
}