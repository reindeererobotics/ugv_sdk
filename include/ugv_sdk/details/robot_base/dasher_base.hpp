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
#include "reindeere/interface/reindeere_message.h"
#include "reindeere/protocol_reindeere/dasher_msg_parser.h"
#include "agilex/interface/agilex_message.h"
#include "reindeere/protocol_reindeere/protocol_reindeere_parser.hpp"

namespace dasher {
class DasherBase : public westonrobot::ScoutBase<westonrobot::ProtocolV2Parser> , public DasherInterface{
 public:
  DasherBase() : westonrobot::ScoutBase<westonrobot::ProtocolV2Parser>(){};
  virtual ~DasherBase() = default;

  bool Connect(std::string can_name) override {
    return ConnectPort(can_name,
                       std::bind(&DasherBase::ParseCANFrame, this,
                                 std::placeholders::_1));
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
    auto common_sensor = AgilexBase<westonrobot::ProtocolV2Parser>::GetCommonSensorStateMsgGroup();

    DasherCommonSensorState dasher_common_sensor;

    dasher_common_sensor.time_stamp = common_sensor.time_stamp;
    dasher_common_sensor.bms_basic_state = common_sensor.bms_basic_state;
    dasher_common_sensor.us_state = common_sensor.us_state;

    dasher_common_sensor.imu_accel_state = common_sensor.imu_accel_state;
    dasher_common_sensor.imu_grav_state = common_sensor.imu_grav_state;
    dasher_common_sensor.imu_gyro_state = common_sensor.imu_gyro_state;
    dasher_common_sensor.imu_mag_state = common_sensor.imu_mag_state;
    dasher_common_sensor.imu_quat_state = common_sensor.imu_quat_state;
    dasher_common_sensor.imu_euler_state = common_sensor.imu_euler_state;

//    std::cout << static_cast<unsigned int>(scout_common_sensor.bms_basic_state.battery_soc) << std::endl;

    return dasher_common_sensor;
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


 protected:
  ReindeereParser parser_;



void ParseCANFrame(can_frame *rx_frame) {
    ReindeereMessage status_msg;

    if (parser_.DecodeMessage(rx_frame, &status_msg)) {
      UpdateRobotCoreState(status_msg);
      UpdateActuatorState(status_msg);
      UpdateCommonSensorState(status_msg);
      UpdateResponseVersion(status_msg);
    }
  }

void UpdateRobotCoreState(const ReindeereMessage &status_msg) {
    std::lock_guard<std::mutex> guard(core_state_mtx_);
    switch (status_msg.type) {
      case AgxMsgSystemState: {
        //   std::cout << "system status feedback received" << std::endl;
        core_state_msgs_.time_stamp = westonrobot::AgxMsgRefClock::now();
        core_state_msgs_.system_state = status_msg.body.system_state_msg;
        break;
      }
      case AgxMsgMotionState: {
        // std::cout << "motion control feedback received" << std::endl;
        core_state_msgs_.time_stamp = westonrobot::AgxMsgRefClock::now();
        core_state_msgs_.motion_state = status_msg.body.motion_state_msg;
        break;
      }
      case AgxMsgLightState: {
        // std::cout << "light control feedback received" << std::endl;
        core_state_msgs_.time_stamp = westonrobot::AgxMsgRefClock::now();
        core_state_msgs_.light_state = status_msg.body.light_state_msg;
        break;
      }
      case AgxMsgMotionModeState: {
        // std::cout << "motion mode feedback received" << std::endl;
        core_state_msgs_.time_stamp = westonrobot::AgxMsgRefClock::now();
        core_state_msgs_.motion_mode_state =
            status_msg.body.motion_mode_state_msg;
        break;
      }
      case AgxMsgRcState: {
        // std::cout << "rc feedback received" << std::endl;
        core_state_msgs_.time_stamp = westonrobot::AgxMsgRefClock::now();
        core_state_msgs_.rc_state = status_msg.body.rc_state_msg;
        break;
      }
      default:
        break;
    }
  }


 void UpdateActuatorState(const ReindeereMessage &status_msg) {
    std::lock_guard<std::mutex> guard(actuator_state_mtx_);
   actuator_state_msgs_.time_stamp = westonrobot::AgxMsgRefClock::now();
    switch (status_msg.type) {
      case AgxMsgMotorAngle: {
        actuator_state_msgs_.motor_angles.angle_5 =
            status_msg.body.motor_angle_msg.angle_5;
        actuator_state_msgs_.motor_angles.angle_6 =
            status_msg.body.motor_angle_msg.angle_6;
        actuator_state_msgs_.motor_angles.angle_7 =
            status_msg.body.motor_angle_msg.angle_7;
        actuator_state_msgs_.motor_angles.angle_8 =
            status_msg.body.motor_angle_msg.angle_8;
        break;
      }
      case AgxMsgMotorSpeed: {
        actuator_state_msgs_.motor_speeds.speed_1 =
            status_msg.body.motor_speed_msg.speed_1;
        actuator_state_msgs_.motor_speeds.speed_2 =
            status_msg.body.motor_speed_msg.speed_2;
        actuator_state_msgs_.motor_speeds.speed_3 =
            status_msg.body.motor_speed_msg.speed_3;
        actuator_state_msgs_.motor_speeds.speed_4 =
            status_msg.body.motor_speed_msg.speed_4;
        break;
      }
      case AgxMsgActuatorHSState: {
        // std::cout << "actuator hs feedback received" << std::endl;
        actuator_state_msgs_.time_stamp = westonrobot::AgxMsgRefClock::now();
        actuator_state_msgs_
            .actuator_hs_state[status_msg.body.actuator_hs_state_msg.motor_id] =
            status_msg.body.actuator_hs_state_msg;
        break;
      }
      case AgxMsgActuatorLSState: {
        // std::cout << "actuator ls feedback received" << std::endl;
        actuator_state_msgs_.time_stamp = westonrobot::AgxMsgRefClock::now();
        actuator_state_msgs_
            .actuator_ls_state[status_msg.body.actuator_ls_state_msg.motor_id] =
            status_msg.body.actuator_ls_state_msg;
        break;
      }
      case AgxMsgActuatorStateV1: {
        // std::cout << "actuator v1 feedback received" << std::endl;
        actuator_state_msgs_.time_stamp = westonrobot::AgxMsgRefClock::now();
        actuator_state_msgs_
            .actuator_state[status_msg.body.v1_actuator_state_msg.motor_id] =
            status_msg.body.v1_actuator_state_msg;
        break;
      }
      default:
        break;
    }
  }


 void UpdateCommonSensorState(const ReindeereMessage &status_msg) {
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
        break;
      }
      case DasherMsgImuAccel: {
        common_sensor_state_msgs_.imu_accel_state =
            status_msg.body.imu_accel_state;
        break;
      }
      case DasherMsgImuEuler: {
        common_sensor_state_msgs_.imu_euler_state =
            status_msg.body.imu_euler_state;
        break;
      }
      case DasherMsgImuGrav: {
        common_sensor_state_msgs_.imu_grav_state =
            status_msg.body.imu_grav_state;
        break;
      }
      case DasherMsgImuGyro: {
        common_sensor_state_msgs_.imu_gyro_state =
            status_msg.body.imu_gyro_state;
        break;
      }
      case DasherMsgImuMag: {
        common_sensor_state_msgs_.imu_mag_state =
            status_msg.body.imu_mag_state;
        break;
      }
      case DasherMsgImuQuat: {
        common_sensor_state_msgs_.imu_quat_state=
            status_msg.body.imu_quat_state;
        break;
      }
      default:
        break;
    }
  }


void UpdateResponseVersion(const ReindeereMessage &status_msg) {
    switch (status_msg.type) {
      case AgxMsgVersionResponse: {
        std::lock_guard<std::mutex> lock(version_str_buf_mtx_);
        for (int i = 0; i < 8; i++) {
          uint8_t data = status_msg.body.version_response_msg.bytes[i];
          if (data < 32 || data > 126) data = 32;
          version_string_buffer_ += data;
        }
        break;
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
