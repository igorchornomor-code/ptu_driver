// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

extern "C"
{
#include "ptu_driver/ptu_sdk.h"
}


#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "ptu_messages/msg/position.hpp"
#include <enums.h>
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ptu_messages/srv/set_stepmode.hpp"


using namespace std::chrono_literals;
ptu_messages::msg::Position getCurrentPosition(ptu_handle* h);

class PTU_driver : public rclcpp::Node
{
  using PositionMsg = ptu_messages::msg::Position;

public:
  PTU_driver()
    : Node("ptu_driver")
  {
    step_mode = CPI_STEP_HALF;
    curPTPosition = { 0, 0 };

    std::string port = this->declare_parameter("port", "/dev/ttyUSB0");
    int err = 0;
    handler = ptu_open(port.c_str(), &err);
    if (err || handler == NULL)
    {
      std::cout << "Failed to create connection to " << port.c_str();
      exit(1);
    }

    this->declare_parameter<int>("time_interval", 100);
    int time_interval_unchecked = this->get_parameter("time_interval").as_int();

    int safe_interval_ms = computeSafeIntervalMs(time_interval_unchecked);
    auto TIME_INTERVAL = std::chrono::milliseconds{ safe_interval_ms };

    RCLCPP_INFO(
      this->get_logger(),
      "Using time_interval = %d ms (requested: %d ms)",
      safe_interval_ms, time_interval_unchecked
    );

    /*
    * -------------------------------------------------
    * -----------------TOPICS--------------------------
    * -------------------------------------------------
    */

    /*
     * Publisher who publishes current position to the subscriber, printing a corresponding message to the console
     * message Type: Position
     * float 64 pan, float 64 tilt
     */
    publisher_get_pos_absolute = this->create_publisher<PositionMsg>("get_position_absolute", 10);
    publisher_get_pos_relative = this->create_publisher<PositionMsg>("get_position_relative", 10);
    publisher_get_speed_absolute = this->create_publisher<PositionMsg>("get_speed_absolute", 10);


    auto timer_callback_publish = [this]() -> void
      {
        auto message1 = getCurrentPosition(handler);
        publisher_get_pos_absolute->publish(message1);

        auto message2 = getPositionRelative(handler);
        publisher_get_pos_relative->publish(message2);

        auto message3 = getSpeedAbsolute(handler);
        publisher_get_speed_absolute->publish(message3);
      };
    timer_publish = this->create_wall_timer(TIME_INTERVAL, timer_callback_publish);


    auto topic_set_pos_abs = [this](const PositionMsg::SharedPtr msg)
      -> void { setPositionAbsolute(msg); };
    subscription_set_pos_absolute = this->create_subscription<PositionMsg>("set_position_absolute", 10, topic_set_pos_abs);

    auto topic_set_pos_rel = [this](const PositionMsg::SharedPtr msg)
      -> void { setPositionRelative(msg); };
    subscription_set_pos_relative = this->create_subscription<PositionMsg>("set_position_relative", 10, topic_set_pos_rel);

    auto topic_set_speed_abs = [this](const PositionMsg::SharedPtr msg)
      -> void { setSpeedAbsolute(msg); };
    subscription_set_speed_absolute = this->create_subscription<PositionMsg>("set_speed_absolute", 10, topic_set_speed_abs);



    /*
    * -------------------------------------------------
    * -------------------Services----------------------
    * -------------------------------------------------
    */

    srv_control_mode_ = this->create_service<std_srvs::srv::SetBool>(
        "set_control_mode",
        [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
            if (req->data) ptu_set_velocity_mode(handler);
            else           ptu_set_position_mode(handler);
            resp->success = true;
        });

    
    srv_control_mode_get_ = this->create_service<std_srvs::srv::Trigger>(
        "get_control_mode",
        [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
          int mode_out = 0;
          int ret = ptu_get_control_mode(handler, &mode_out);

          if (ret != 0) {
            resp->success = false;
            resp->message = "Failed to read control mode";
            return;
          }

          std::string message;
          switch (mode_out) {
              case 1: {
                message = "PTU is in control mode";
                break;
              }
              case 2: {
                message = "PTU is in velocity mode";
                break;
              }
              default: {
                message = "PTU control mode: " + std::to_string(mode_out);
                break;
              }
          }
          resp->success = true;
          resp->message = message;
        });

    srv_step_mode_ = this->create_service<ptu_messages::srv::SetStepmode>(
        "set_step_mode",
        [this](
            const std::shared_ptr<ptu_messages::srv::SetStepmode::Request> req,
            std::shared_ptr<ptu_messages::srv::SetStepmode::Response> resp) {
            step_mode = (cpi_stepmode)req->step_mode;

            ptu_set_step_mode(handler, step_mode);
            ptu_reset_home(handler, CPI_RESET_ALL);
            ptu_await(handler);
            resp->success = true;
        });

    srv_reset_home_ = this->create_service<std_srvs::srv::Trigger>(
        "reset_home",
        [this](
            const std::shared_ptr<std_srvs::srv::Trigger::Request> ,
            std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
            ptu_reset_home(handler, CPI_RESET_ALL);
            ptu_await(handler);
            resp->success = true;
        });

    // PP SET
    int ticks = 400;
    ptu_set_pan_abs(handler, ticks);
  }

private:
  struct PTPosition {
    int pan;
    int tilt;
  };
  rclcpp::TimerBase::SharedPtr timer_publish;
  
  rclcpp::Publisher<PositionMsg>::SharedPtr publisher_get_pos_absolute;
  rclcpp::Publisher<PositionMsg>::SharedPtr publisher_get_pos_relative;
  rclcpp::Publisher<PositionMsg>::SharedPtr publisher_get_speed_absolute;

  rclcpp::Subscription<PositionMsg>::SharedPtr subscription_set_pos_absolute;
  rclcpp::Subscription<PositionMsg>::SharedPtr subscription_set_pos_relative;
  rclcpp::Subscription<PositionMsg>::SharedPtr subscription_set_speed_absolute;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_control_mode_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_control_mode_get_;
  rclcpp::Service<ptu_messages::srv::SetStepmode>::SharedPtr srv_step_mode_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_home_;

  ptu_handle* handler;
  cpi_stepmode step_mode;
  PTPosition curPTPosition = { 0, 0 };

  // enum cpi_stepmode {
  //     CPI_STEP_FULL, CPI_STEP_HALF, CPI_STEP_QUARTER, CPI_STEP_AUTO
  // };
  struct AxisLimits
  {
    int pan_min;
    int pan_max;
    int tilt_min;
    int tilt_max;
  };
  std::unordered_map<cpi_stepmode, AxisLimits> stepModeLimits = {
    {CPI_STEP_FULL, {-3499, 3500, -3499, 1166}},
    {CPI_STEP_HALF, {-6999, 7000, -6999, 2333}},
    {CPI_STEP_QUARTER, {-13999, 14000, -13996, 4664}},
    {CPI_STEP_AUTO, {-27999, 28000, -27996, 9332}} };
  std::unordered_map<cpi_stepmode, double> panStepSize = {
    {CPI_STEP_FULL, 360.0 / 7000.0},
    {CPI_STEP_HALF, 360.0 / 14000.0},
    {CPI_STEP_QUARTER, 360.0 / 28000.0},
    {CPI_STEP_AUTO, 360.0 / 56000.0} };
  std::unordered_map<cpi_stepmode, double> tiltStepSize = {
    {CPI_STEP_FULL, 120.0 / 4666.0},
    {CPI_STEP_HALF, 120.0 / 9333.0},
    {CPI_STEP_QUARTER, 120.0 / 18661.0},
    {CPI_STEP_AUTO, 120.0 / 37329.0} };





  double getStepSize(const std::string& axis) const
  {
    if (axis == "pan")
    {
      auto it = panStepSize.find(step_mode);
      if (it != panStepSize.end())
      {
        return it->second;
      }
    }
    else if (axis == "tilt")
    {
      auto it = tiltStepSize.find(step_mode);
      if (it != tiltStepSize.end())
      {
        return it->second;
      }
    }

    std::cerr << "Invalid axis or step mode: " << axis << ", " << step_mode << std::endl;
    return 0.0;
  }

  int convertAngleToSteps(double angle, std::string axis)
  {
    if (axis != "pan" && axis != "tilt")
    {
      std::cerr << "Invalid axis:" << axis << std::endl;
      return 12345678;
    }
    int steps = round(angle / getStepSize(axis));
    return steps;
  }

  double convertStepsToAngle(double steps, std::string axis)
  {
    double angle = steps * getStepSize(axis);
    return angle;
  }

  int computeSafeIntervalMs(int requested_ms)
{
  using namespace std::chrono;
  if (requested_ms <= 0) {
    requested_ms = 1;
  }

  auto start = steady_clock::now();
  (void)getCurrentPosition(handler);
  (void)getPositionRelative(handler);
  (void)getSpeedAbsolute(handler);
  auto end = steady_clock::now();

  auto duration_ms = duration_cast<milliseconds>(end - start).count();
  if (duration_ms <= 0) {
    duration_ms = 1;
  }

  long long min_interval_ms_ll = static_cast<long long>(std::ceil(duration_ms * 1.5));
  int min_interval_ms = static_cast<int>(min_interval_ms_ll);

  if (requested_ms < min_interval_ms) {
    RCLCPP_WARN(
      this->get_logger(),
      "Requested time_interval=%d ms is too small. "
      "getCurrentPosition() took %lld ms, using %d ms instead.",
      requested_ms, duration_ms, min_interval_ms
    );
    return min_interval_ms;
  }

  return requested_ms;
}



  PositionMsg getCurrentPosition(ptu_handle* h)
  {
    auto message = ptu_messages::msg::Position();

    int ticks;
    if (ptu_get_pan_pos(h, &ticks) == 0)
    {
      curPTPosition.pan = ticks;
      message.pan = static_cast<double>(ticks);
    } else {
      message.pan = 0.0;
    }

    if (ptu_get_tilt_pos(h, &ticks) == 0) {
      curPTPosition.tilt = ticks;
      message.tilt = static_cast<double>(ticks);
    } else {
      message.tilt = 0.0;
    }

    message.pan = convertStepsToAngle(message.pan, "pan");
    message.tilt = convertStepsToAngle(message.tilt, "tilt");
    return message;
  }


  PositionMsg getPositionRelative(ptu_handle* h)
  {
    auto message = ptu_messages::msg::Position();

    int ticks;
    if (ptu_get_pan_pos_rel(h, &ticks) == 0)
    {
      message.pan = static_cast<double>(ticks);
    } else {
      message.pan = 0.0;
    }

    if (ptu_get_tilt_pos_rel(h, &ticks) == 0) {
      message.tilt = static_cast<double>(ticks);
    } else {
      message.tilt = 0.0;
    }

    message.pan = convertStepsToAngle(message.pan, "pan");
    message.tilt = convertStepsToAngle(message.tilt, "tilt");
    return message;
  }

  PositionMsg getSpeedAbsolute(ptu_handle* h)
  {
    auto message = ptu_messages::msg::Position();

    int ticks_per_s_out = 0;
    if (ptu_get_pan_vel(h, &ticks_per_s_out) == 0)
    {
      message.pan = static_cast<double>(ticks_per_s_out);
    } else {
      message.pan = 0.0;
    }

    if (ptu_get_tilt_vel(h, &ticks_per_s_out) == 0) {
      message.tilt = static_cast<double>(ticks_per_s_out);
    } else {
      message.tilt = 0.0;
    }

    message.pan = convertStepsToAngle(message.pan, "pan");
    message.tilt = convertStepsToAngle(message.tilt, "tilt");
    return message;
  }





  void setPositionAbsolute(PositionMsg::SharedPtr msg) {
    int panInSteps = 0;
    int tiltInSteps = 0;

    if (std::isnan(msg->pan)) {
      panInSteps = curPTPosition.pan;
    }
    else {
      panInSteps = convertAngleToSteps(msg->pan, "pan");
    }

    if (std::isnan(msg->tilt)) {
      tiltInSteps = curPTPosition.tilt;
    }
    else {
      tiltInSteps = convertAngleToSteps(msg->tilt, "tilt");
    }

    ptu_set_pan_abs(handler, panInSteps);
    ptu_set_tilt_abs(handler, tiltInSteps);
  }

  void setPositionRelative(PositionMsg::SharedPtr msg) {
    int panInSteps = 0;
    int tiltInSteps = 0;

    if (std::isnan(msg->pan)) {
      panInSteps = 0;
    }
    else {
      panInSteps = convertAngleToSteps(msg->pan, "pan");
    }

    if (std::isnan(msg->tilt)) {
      tiltInSteps = 0;
    }
    else {
      tiltInSteps = convertAngleToSteps(msg->tilt, "tilt");
    }

    ptu_set_pan_rel(handler, panInSteps);
    ptu_set_tilt_rel(handler, tiltInSteps);
  }



  void setSpeedAbsolute(PositionMsg::SharedPtr msg) {
    int pan_ticks_per_s = 0;
    int tilt_ticks_per_s = 0;

    if (std::isnan(msg->pan)) {
      pan_ticks_per_s = curPTPosition.pan;
    }
    else {
      pan_ticks_per_s = convertAngleToSteps(msg->pan, "pan");
    }

    if (std::isnan(msg->tilt)) {
      tilt_ticks_per_s = curPTPosition.tilt;
    }
    else {
      tilt_ticks_per_s = convertAngleToSteps(msg->tilt, "tilt");
    }

    if (ptu_set_pan_vel(handler, pan_ticks_per_s) != 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to set pan speed: %.5f! \nProbably the given value is outside the speed bounds.", msg->pan
      );
    }

    if (ptu_set_tilt_vel(handler, tilt_ticks_per_s) != 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to set tilt speed: %.5f! \nProbably the given value is outside the speed bounds.", msg->tilt
      );
    }
  }



};



int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PTU_driver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
